package com.team254.frc2016;

import java.util.Map;

import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.Pose2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

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
 * @author Jared
 */
public class RobotState {
    public static final int kObservationBufferSize = 100;

    public static final Pose2d kVehicleToTurretFixed = new Pose2d(
            new Translation2d(Constants.kTurretXOffset, Constants.kTurretYOffset),
            Rotation2d.fromDegrees(Constants.kTurretAngleOffsetDegrees));

    public static final Pose2d kTurretRotatingToCamera = new Pose2d(
            new Translation2d(Constants.kCameraXOffset, Constants.kCameraYOffset),
            Rotation2d.fromDegrees(Constants.kCameraAngleOffsetDegrees));

    // nanoTime -> Pose2d or Rotation2d
    protected InterpolatingTreeMap<Long, Pose2d> odometric_to_vehicle_;
    protected InterpolatingTreeMap<Long, Rotation2d> turret_rotation_;

    public RobotState(long start_time, Pose2d initial_odometric_to_vehicle, Rotation2d initial_turret_rotation) {
        odometric_to_vehicle_ = new InterpolatingTreeMap<Long, Pose2d>(kObservationBufferSize);
        odometric_to_vehicle_.put(start_time, initial_odometric_to_vehicle);
        turret_rotation_ = new InterpolatingTreeMap<Long, Rotation2d>(kObservationBufferSize);
        turret_rotation_.put(start_time, initial_turret_rotation);
    }

    public synchronized Pose2d getOdometricToVehicle(long timestamp) {
        return odometric_to_vehicle_.getInterpolated(timestamp);
    }

    public synchronized Map.Entry<Long, Pose2d> getLatestOdometricToVehicle() {
        return odometric_to_vehicle_.lastEntry();
    }

    public synchronized Rotation2d getTurretRotation(long timestamp) {
        return turret_rotation_.getInterpolated(timestamp);
    }

    public synchronized Map.Entry<Long, Rotation2d> getLatestTurretRotation() {
        return turret_rotation_.lastEntry();
    }

    public synchronized Pose2d getOdometricToTurretRotated(long timestamp) {
        return odometric_to_vehicle_.getInterpolated(timestamp).transformBy(kVehicleToTurretFixed)
                .transformBy(Pose2d.fromRotation(turret_rotation_.getInterpolated(timestamp)));
    }

    public synchronized Pose2d getOdometricToCamera(long timestamp) {
        return getOdometricToTurretRotated(timestamp).transformBy(kTurretRotatingToCamera);
    }

    public synchronized void addOdometricToVehicleObservation(long timestamp, Pose2d observation) {
        odometric_to_vehicle_.put(timestamp, observation);
    }

    public synchronized void addTurretRotationObservation(long timestamp, Rotation2d observation) {
        turret_rotation_.put(timestamp, observation);
    }

    public synchronized void addObservations(long timestamp, Pose2d odometric_to_vehicle, Rotation2d turret_rotation) {
        addOdometricToVehicleObservation(timestamp, odometric_to_vehicle);
        addTurretRotationObservation(timestamp, turret_rotation);
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
}
