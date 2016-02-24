package com.team254.frc2016;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;

import com.team254.frc2016.GoalTracker.TrackReport;
import com.team254.frc2016.subsystems.Shooter;
import com.team254.frc2016.vision.TargetInfo;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * RobotState keeps track of the poses of various inertial frames throughout the
 * match.
 * 
 * Robot frames of interest (from parent to child):
 * 
 * 1. Odometric frame: origin is where the robot is turned on
 * 
 * 2. Vehicle frame: origin is the center of the robot wheelbase, facing
 * forwards
 * 
 * 3. Turret fixed frame: origin is the center of the turret when the turret is
 * at 0 degrees rotation
 * 
 * 4. Turret rotating frame: origin is the center of the turret as it rotates
 * 
 * 5. Camera frame: origin is the center of the camera imager as it rotates with
 * the turret
 * 
 * 6. Goal frame: origin is the center of the goal (note that orientation in
 * this frame is arbitrary). Also note that there can be multiple goal frames.
 * 
 * As a simple kinematic chain with 5 frames, there are 4 transforms of
 * interest:
 * 
 * 1. Odometric-to-vehicle: This is tracked over time by integrating encoder and
 * gyro measurements. It will inevitably drift, but is usually accurate over
 * short time periods.
 * 
 * 2. Vehicle-to-turret-fixed: This is a constant.
 * 
 * 3. Vehicle-to-turret-rotating: This is a pure rotation, and is tracked over
 * time.
 * 
 * 4. Turret-rotating-to-camera: This is a constant.
 * 
 * 5. Camera-to-goal: This is a pure translation, and is measured by the vision
 * system.
 * 
 * @author Jared
 */
public class RobotState {
    private static RobotState instance_ = new RobotState();

    public static RobotState getInstance() {
        return instance_;
    }

    public static final int kObservationBufferSize = 100;
    public static final double kMaxTargetAge = 0.4;

    public static final RigidTransform2d kVehicleToTurretFixed = new RigidTransform2d(
            new Translation2d(Constants.kTurretXOffset, Constants.kTurretYOffset),
            Rotation2d.fromDegrees(Constants.kTurretAngleOffsetDegrees));

    public static final RigidTransform2d kTurretRotatingToCamera = new RigidTransform2d(
            new Translation2d(Constants.kCameraXOffset, Constants.kCameraYOffset),
            Rotation2d.fromDegrees(Constants.kCameraAngleOffsetDegrees));

    // FPGATimestamp -> Pose2d or Rotation2d
    protected InterpolatingTreeMap<InterpolatingDouble, RigidTransform2d> odometric_to_vehicle_;
    protected InterpolatingTreeMap<InterpolatingDouble, Rotation2d> turret_rotation_;
    protected GoalTracker goal_tracker_;
    protected double latest_camera_to_goals_detected_timestamp_;
    protected double latest_camera_to_goals_undetected_timestamp_;
    protected Rotation2d camera_pitch_correction_;
    protected double differential_height_;

    protected RobotState() {
        reset(0, new RigidTransform2d(), new Rotation2d());
    }

    public synchronized void reset(double start_time, RigidTransform2d initial_odometric_to_vehicle,
            Rotation2d initial_turret_rotation) {
        odometric_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        odometric_to_vehicle_.put(new InterpolatingDouble(start_time), initial_odometric_to_vehicle);
        turret_rotation_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        turret_rotation_.put(new InterpolatingDouble(start_time), initial_turret_rotation);
        goal_tracker_ = new GoalTracker();
        latest_camera_to_goals_detected_timestamp_ = 0;
        latest_camera_to_goals_undetected_timestamp_ = start_time;
        camera_pitch_correction_ = Rotation2d.fromDegrees(-Constants.kCameraPitchAngleDegrees);
        differential_height_ = Constants.kCenterOfTargetHeight - Constants.kCameraZOffset;
    }

    public synchronized RigidTransform2d getOdometricToVehicle(double timestamp) {
        return odometric_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, RigidTransform2d> getLatestOdometricToVehicle() {
        return odometric_to_vehicle_.lastEntry();
    }

    public synchronized Rotation2d getTurretRotation(double timestamp) {
        return turret_rotation_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Rotation2d> getLatestTurretRotation() {
        return turret_rotation_.lastEntry();
    }

    public synchronized RigidTransform2d getOdometricToTurretRotated(double timestamp) {
        InterpolatingDouble key = new InterpolatingDouble(timestamp);
        return odometric_to_vehicle_.getInterpolated(key).transformBy(kVehicleToTurretFixed)
                .transformBy(RigidTransform2d.fromRotation(turret_rotation_.getInterpolated(key)));
    }

    public synchronized RigidTransform2d getOdometricToCamera(double timestamp) {
        return getOdometricToTurretRotated(timestamp).transformBy(kTurretRotatingToCamera);
    }

    public synchronized List<RigidTransform2d> getCaptureTimeOdometricToGoal() {
        List<RigidTransform2d> rv = new ArrayList<>();
        for (TrackReport report : goal_tracker_.getTracks()) {
            rv.add(RigidTransform2d.fromTranslation(report.odometric_to_goal));
        }
        return rv;
    }

    public synchronized List<Shooter.AimingParameters> getAimingParameters(double current_timestamp,
            Comparator<TrackReport> comparator) {
        List<Shooter.AimingParameters> rv = new ArrayList<>();
        if (current_timestamp - latest_camera_to_goals_detected_timestamp_ > kMaxTargetAge) {
            return rv;
        }
        List<TrackReport> reports = goal_tracker_.getTracks();
        Collections.sort(reports, comparator);

        // TODO: simplify this math
        RigidTransform2d latest_turret_fixed_to_capture_time_turret_fixed = getLatestOdometricToVehicle().getValue()
                .transformBy(kVehicleToTurretFixed).inverse()
                .transformBy(getOdometricToVehicle(latest_camera_to_goals_detected_timestamp_)
                        .transformBy(kVehicleToTurretFixed));

        for (TrackReport report : reports) {
            RigidTransform2d latest_turret_fixed_to_goal = latest_turret_fixed_to_capture_time_turret_fixed
                    .transformBy(kVehicleToTurretFixed.inverse())
                    .transformBy(getOdometricToVehicle(latest_camera_to_goals_detected_timestamp_).inverse())
                    .transformBy(RigidTransform2d.fromTranslation(report.odometric_to_goal));

            // We can actually disregard the angular portion of this pose. It is
            // the bearing that we care about!
            rv.add(new Shooter.AimingParameters(latest_turret_fixed_to_goal.getTranslation().norm(),
                    new Rotation2d(latest_turret_fixed_to_goal.getTranslation().getX(),
                            latest_turret_fixed_to_goal.getTranslation().getY(), true),
                    report.id));
        }
        return rv;
    }

    public synchronized void addOdometricToVehicleObservation(double timestamp, RigidTransform2d observation) {
        odometric_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addTurretRotationObservation(double timestamp, Rotation2d observation) {
        turret_rotation_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, RigidTransform2d odometric_to_vehicle,
            Rotation2d turret_rotation) {
        addOdometricToVehicleObservation(timestamp, odometric_to_vehicle);
        addTurretRotationObservation(timestamp, turret_rotation);
    }

    public synchronized void addVisionUpdate(double timestamp, List<TargetInfo> vision_update) {
        List<Translation2d> odometric_to_goals = new ArrayList<>();
        RigidTransform2d odometric_to_camera = getOdometricToCamera(timestamp);
        if (vision_update == null || vision_update.isEmpty()) {
            latest_camera_to_goals_undetected_timestamp_ = timestamp;
        } else {
            latest_camera_to_goals_detected_timestamp_ = timestamp;
            for (TargetInfo target : vision_update) {
                // Compensate for camera pitch
                double xr = target.getZ() * camera_pitch_correction_.sin()
                        + target.getX() * camera_pitch_correction_.cos();
                double yr = target.getY();
                double zr = target.getZ() * camera_pitch_correction_.cos()
                        - target.getX() * camera_pitch_correction_.sin();

                // find intersection with the goal
                if (zr > 0) {
                    double scaling = differential_height_ / zr;
                    double distance = Math.hypot(xr, yr) * scaling;
                    Rotation2d angle = new Rotation2d(xr, yr, true);
                    odometric_to_goals.add(odometric_to_camera
                            .transformBy(RigidTransform2d
                                    .fromTranslation(new Translation2d(distance * angle.cos(), distance * angle.sin())))
                            .getTranslation());
                }
            }
        }
        goal_tracker_.update(timestamp, odometric_to_goals);
    }

    public synchronized void resetVision() {
        latest_camera_to_goals_detected_timestamp_ = 0;
        goal_tracker_.reset();
    }

    public RigidTransform2d generateOdometryFromSensors(double left_encoder_delta_distance,
            double right_encoder_delta_distance, Rotation2d current_gyro_angle) {
        RigidTransform2d last_measurement = getLatestOdometricToVehicle().getValue();
        RigidTransform2d differential_pose = new RigidTransform2d(
                new Translation2d((left_encoder_delta_distance + right_encoder_delta_distance) / 2, 0),
                last_measurement.getRotation().inverse().rotateBy(current_gyro_angle));
        RigidTransform2d new_observation = last_measurement.transformBy(differential_pose);
        new_observation.getRotation().normalize();
        return new_observation;
    }

    public void outputToSmartDashboard() {
        RigidTransform2d odometry = getLatestOdometricToVehicle().getValue();
        SmartDashboard.putNumber("robot_pose_x", odometry.getTranslation().getX());
        SmartDashboard.putNumber("robot_pose_y", odometry.getTranslation().getY());
        SmartDashboard.putNumber("robot_pose_theta", odometry.getRotation().getDegrees());
        List<RigidTransform2d> poses = getCaptureTimeOdometricToGoal();
        for (RigidTransform2d pose : poses) {
            // Only output first goal
            SmartDashboard.putNumber("goal_pose_x", pose.getTranslation().getX());
            SmartDashboard.putNumber("goal_pose_y", pose.getTranslation().getY());
            break;
        }
    }
}
