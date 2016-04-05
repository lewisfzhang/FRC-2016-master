package com.team254.lib.util;

import java.util.Optional;

public class AdaptivePurePursuitController {
    private static final double kEpsilon = 1E-9;

    double mFixedLookahead;
    Path mPath;
    Command mLastCommand;
    double mLastTime;
    double mMaxAccel;
    double mDt;

    public AdaptivePurePursuitController(double fixed_lookahead, double max_accel, double nominal_dt, Path path) {
        mFixedLookahead = fixed_lookahead;
        mMaxAccel = max_accel;
        mPath = path;
        mDt = nominal_dt;
        mLastCommand = null;
    }

    public boolean isDone() {
        return mPath.getRemainingLength() == 0;
    }

    public static class Command {
        public final double linear_velocity;
        public final double angular_velocity;

        public Command(double linear_velocity, double angular_velocity) {
            this.linear_velocity = linear_velocity;
            this.angular_velocity = angular_velocity;
        }
    }

    public Command update(RigidTransform2d robot_pose, double now) {
        double distance_from_path = mPath.update(robot_pose.getTranslation());
        PathSegment.Sample lookahead_point = mPath.getLookaheadPoint(robot_pose.getTranslation(),
                distance_from_path + mFixedLookahead);
        Optional<Circle> circle = joinPath(robot_pose, lookahead_point.translation);
        // System.out.println("Pose is " + robot_pose + " and lookahead point is
        // " + lookahead_point.translation);

        double speed = Math.abs(lookahead_point.speed);
        // Ensure we don't accelerate too fast from the previous command
        double dt = now - mLastTime;
        if (mLastCommand == null) {
            mLastCommand = new Command(0, 0);
            dt = mDt;
        }
        double accel = (speed - mLastCommand.linear_velocity) / dt;
        if (accel < -mMaxAccel) {
            speed = mLastCommand.linear_velocity - mMaxAccel * dt;
        } else if (accel > mMaxAccel) {
            speed = mLastCommand.linear_velocity + mMaxAccel * dt;
        }

        // Ensure we slow down in time to stop
        // vf^2 = v^2 + 2*a*d
        // 0 = v^2 + 2*a*d
        double remaining_distance = mPath.getRemainingLength();
        double max_allowed_speed = Math.sqrt(2 * mMaxAccel * remaining_distance);
        if (speed > max_allowed_speed) {
            speed = max_allowed_speed;
        }

        Command rv;
        if (circle.isPresent()) {
            rv = new Command(speed, (circle.get().turn_right ? -1 : 1) * speed / circle.get().radius);
        } else {
            rv = new Command(speed, 0.0);
        }
        mLastTime = now;
        mLastCommand = rv;
        return rv;
    }

    public static class Circle {
        public final Translation2d center;
        public final double radius;
        public final boolean turn_right;

        public Circle(Translation2d center, double radius, boolean turn_right) {
            this.center = center;
            this.radius = radius;
            this.turn_right = turn_right;
        }
    }

    public static Optional<Circle> joinPath(RigidTransform2d robot_pose, Translation2d lookahead_point) {
        double x1 = robot_pose.getTranslation().getX();
        double y1 = robot_pose.getTranslation().getY();
        double x2 = lookahead_point.getX();
        double y2 = lookahead_point.getY();

        Translation2d pose_to_lookahead = robot_pose.getTranslation().inverse().translateBy(lookahead_point);
        double cross_product = pose_to_lookahead.getX() * robot_pose.getRotation().sin()
                - pose_to_lookahead.getY() * robot_pose.getRotation().cos();
        if (Math.abs(cross_product) < kEpsilon) {
            return Optional.empty();
        }

        double dx = x1 - x2;
        double dy = y1 - y2;
        double my = (cross_product > 0 ? -1 : 1) * robot_pose.getRotation().cos();
        double mx = (cross_product > 0 ? 1 : -1) * robot_pose.getRotation().sin();

        double cross_term = mx * dx + my * dy;
        // System.out.println("center to right? " + (cross_product > 0) + ", dx
        // " + dx + ", dy " + dy + ", cross term "
        // + cross_term + ", pose " + robot_pose + ", lookahead " +
        // lookahead_point);
        if (Math.abs(cross_term) < kEpsilon) {
            // Points are colinear
            return Optional.empty();
        }

        return Optional.of(new Circle(
                new Translation2d((mx * (x1 * x1 - x2 * x2 - dy * dy) + 2 * my * x1 * dy) / (2 * cross_term),
                        (-my * (-y1 * y1 + y2 * y2 + dx * dx) + 2 * mx * y1 * dx) / (2 * cross_term)),
                .5 * Math.abs((dx * dx + dy * dy) / cross_term), cross_product > 0));
    }

}
