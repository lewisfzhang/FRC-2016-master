package com.team254.frc2016;

import com.team254.frc2016.subsystems.Drive;
import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

public class Kinematics {

    public static RigidTransform2d forwardKinematics(double left_encoder_delta_distance,
            double right_encoder_delta_distance, Rotation2d delta_rotation) {
        return new RigidTransform2d(
                new Translation2d((left_encoder_delta_distance + right_encoder_delta_distance) / 2, 0), delta_rotation);
    }

    public static RigidTransform2d integrateForwardKinematics(RigidTransform2d current_pose,
            double left_encoder_delta_distance, double right_encoder_delta_distance, Rotation2d current_heading) {
        return current_pose.transformBy(forwardKinematics(left_encoder_delta_distance, right_encoder_delta_distance,
                current_pose.getRotation().inverse().rotateBy(current_heading)));
    }

    public static Drive.VelocityHeadingSetpoint inverseKinematics(double linear_velocity, double curvature,
            Rotation2d start_heading, double start_time) {
        // From linear velocity and curvature, compute left velocity, right velocity, and heading velocity.
        
        
        Drive.VelocityHeadingSetpoint setpoint = null;
        return setpoint;
    }
}
