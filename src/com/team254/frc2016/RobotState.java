package com.team254.frc2016;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import com.team254.frc2016.subsystems.Shooter;
import com.team254.frc2016.vision.TargetInfo;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.Pose2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

import edu.wpi.first.wpilibj.Timer;
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

    public static final Pose2d kVehicleToTurretFixed = new Pose2d(
            new Translation2d(Constants.kTurretXOffset, Constants.kTurretYOffset),
            Rotation2d.fromDegrees(Constants.kTurretAngleOffsetDegrees));

    public static final Pose2d kTurretRotatingToCamera = new Pose2d(
            new Translation2d(Constants.kCameraXOffset, Constants.kCameraYOffset),
            Rotation2d.fromDegrees(Constants.kCameraAngleOffsetDegrees));

    // FPGATimestamp -> Pose2d or Rotation2d
    protected InterpolatingTreeMap<InterpolatingDouble, Pose2d> odometric_to_vehicle_;
    protected InterpolatingTreeMap<InterpolatingDouble, Rotation2d> turret_rotation_;
    protected List<Translation2d> camera_to_goals_;
    protected double latest_camera_to_goals_detected_timestamp_;
    protected double latest_camera_to_goals_undetected_timestamp_;
    protected Rotation2d camera_pitch_correction_;
    protected double differential_height_;

    protected RobotState() {
        reset(0, new Pose2d(), new Rotation2d());
    }

    public synchronized void reset(double start_time, Pose2d initial_odometric_to_vehicle,
            Rotation2d initial_turret_rotation) {
        odometric_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        odometric_to_vehicle_.put(new InterpolatingDouble(start_time), initial_odometric_to_vehicle);
        turret_rotation_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        turret_rotation_.put(new InterpolatingDouble(start_time), initial_turret_rotation);
        camera_to_goals_ = new ArrayList<Translation2d>();
        latest_camera_to_goals_detected_timestamp_ = 0;
        latest_camera_to_goals_undetected_timestamp_ = start_time;
        camera_pitch_correction_ = Rotation2d.fromDegrees(-Constants.kCameraPitchAngleDegrees);
        differential_height_ = Constants.kCenterOfTargetHeight - Constants.kCameraZOffset;
    }

    public synchronized Pose2d getOdometricToVehicle(double timestamp) {
        return odometric_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestOdometricToVehicle() {
        return odometric_to_vehicle_.lastEntry();
    }

    public synchronized Rotation2d getTurretRotation(double timestamp) {
        return turret_rotation_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Rotation2d> getLatestTurretRotation() {
        return turret_rotation_.lastEntry();
    }

    public synchronized Pose2d getOdometricToTurretRotated(double timestamp) {
        InterpolatingDouble key = new InterpolatingDouble(timestamp);
        return odometric_to_vehicle_.getInterpolated(key).transformBy(kVehicleToTurretFixed)
                .transformBy(Pose2d.fromRotation(turret_rotation_.getInterpolated(key)));
    }

    public synchronized Pose2d getOdometricToCamera(double timestamp) {
        return getOdometricToTurretRotated(timestamp).transformBy(kTurretRotatingToCamera);
    }

    public synchronized boolean canSeeTarget() {
        return latest_camera_to_goals_detected_timestamp_ > latest_camera_to_goals_undetected_timestamp_;
    }

    public synchronized List<Pose2d> getCaptureTimeOdometricToGoals() {
        List<Pose2d> rv = new ArrayList<>();

        Pose2d odometric_to_camera = getOdometricToCamera(latest_camera_to_goals_detected_timestamp_);
        for (Translation2d pos : camera_to_goals_) {
            rv.add(odometric_to_camera.transformBy(Pose2d.fromTranslation(pos)));
        }
        return rv;
    }

    public synchronized List<Shooter.AimingParameters> getAimingParameters() {
        List<Shooter.AimingParameters> rv = new ArrayList<>();
        if (Timer.getFPGATimestamp() - latest_camera_to_goals_detected_timestamp_ > kMaxTargetAge) {
            return rv;
        }
        Pose2d capture_time_turret_fixed_to_camera = Pose2d
                .fromRotation(getTurretRotation(latest_camera_to_goals_detected_timestamp_ - Constants.kAutoAimLagTime))
                .transformBy(kTurretRotatingToCamera);
        Pose2d latest_turret_fixed_to_capture_time_turret_fixed = getLatestOdometricToVehicle().getValue()
                .transformBy(kVehicleToTurretFixed).inverse().transformBy(
                        getOdometricToVehicle(latest_camera_to_goals_detected_timestamp_ - Constants.kAutoAimLagTime)
                                .transformBy(kVehicleToTurretFixed));
        for (Translation2d pos : camera_to_goals_) {
            Pose2d capture_time_turret_fixed_to_goal = capture_time_turret_fixed_to_camera
                    .transformBy(Pose2d.fromTranslation(pos));
            Pose2d latest_turret_fixed_to_goal = latest_turret_fixed_to_capture_time_turret_fixed
                    .transformBy(capture_time_turret_fixed_to_goal);

            // We can actually disregard the angular portion of this pose. It is
            // the bearing that we care about!
            rv.add(new Shooter.AimingParameters(latest_turret_fixed_to_goal.getTranslation().norm(),
                    new Rotation2d(latest_turret_fixed_to_goal.getTranslation().getX(),
                            latest_turret_fixed_to_goal.getTranslation().getY(), true)));
        }
        return rv;
    }

    public synchronized void addOdometricToVehicleObservation(double timestamp, Pose2d observation) {
        odometric_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addTurretRotationObservation(double timestamp, Rotation2d observation) {
        turret_rotation_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Pose2d odometric_to_vehicle,
            Rotation2d turret_rotation) {
        addOdometricToVehicleObservation(timestamp, odometric_to_vehicle);
        addTurretRotationObservation(timestamp, turret_rotation);
    }

    public synchronized void addVisionUpdate(double timestamp, List<TargetInfo> vision_update) {
        if (vision_update == null || vision_update.isEmpty()) {
            latest_camera_to_goals_undetected_timestamp_ = timestamp;
        } else {
            latest_camera_to_goals_detected_timestamp_ = timestamp;
            camera_to_goals_.clear();
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
                    camera_to_goals_.add(new Translation2d(distance * angle.cos(), distance * angle.sin()));
                }
            }
        }
    }

    public synchronized void resetVision() {
        latest_camera_to_goals_detected_timestamp_ = 0;
    }

    public Pose2d generateOdometryFromSensors(double left_encoder_delta_distance, double right_encoder_delta_distance,
            Rotation2d current_gyro_angle) {
        Pose2d last_measurement = getLatestOdometricToVehicle().getValue();
        Pose2d differential_pose = new Pose2d(
                new Translation2d((left_encoder_delta_distance + right_encoder_delta_distance) / 2, 0),
                last_measurement.getRotation().inverse().rotateBy(current_gyro_angle));
        Pose2d new_observation = last_measurement.transformBy(differential_pose);
        new_observation.getRotation().normalize();
        return new_observation;
    }

    public void outputToSmartDashboard() {
        Pose2d odometry = getLatestOdometricToVehicle().getValue();
        SmartDashboard.putNumber("robot_pose_x", odometry.getTranslation().getX());
        SmartDashboard.putNumber("robot_pose_y", odometry.getTranslation().getY());
        SmartDashboard.putNumber("robot_pose_theta", odometry.getRotation().getDegrees());
        if (camera_to_goals_.size() > 0) {
            List<Pose2d> poses = getCaptureTimeOdometricToGoals();
            SmartDashboard.putNumber("goal_pose_x", poses.get(0).getTranslation().getX());
            SmartDashboard.putNumber("goal_pose_y", poses.get(0).getTranslation().getY());
        }
    }
}
