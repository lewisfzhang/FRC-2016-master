package com.team254.frc2016;

import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

public class Kinematics {
    private static final double kEpsilon = 1E-9;

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

    public static class DriveVelocity {
        public final double left;
        public final double right;

        public DriveVelocity(double left, double right) {
            this.left = left;
            this.right = right;
        }
    }

    public static DriveVelocity inverseKinematics(double linear_velocity, double angular_velocity) {
        if (Math.abs(angular_velocity) < kEpsilon) {
            return new DriveVelocity(linear_velocity, linear_velocity);
        }
        // From linear velocity and curvature, compute left velocity, right
        // velocity, and heading velocity.
        double delta_v = Constants.kTrackWidthInches / 2 * angular_velocity / Constants.kTrackScrubFactor;

        // Adjust linear velocity if either side is moving too fast
        if (linear_velocity - delta_v < -Constants.kPathFollowingMaxVel) {
            linear_velocity = -Constants.kPathFollowingMaxVel + delta_v;
        } else if (linear_velocity + delta_v > Constants.kPathFollowingMaxVel) {
            linear_velocity = Constants.kPathFollowingMaxVel - delta_v;
        }
        return new DriveVelocity(linear_velocity - delta_v, linear_velocity + delta_v);
    }
}
