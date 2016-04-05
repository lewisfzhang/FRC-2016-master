package com.team254.lib.util;

import java.util.Optional;

public class AdaptivePurePursuitController {
    private static final double kEpsilon = 1E-9;

    double mFixedLookahead;
    Path mPath;

    public AdaptivePurePursuitController(double fixed_lookahead, Path path) {
        mFixedLookahead = fixed_lookahead;
        mPath = path;
    }

    public boolean isDone() {
        return mPath.getRemainingLength() == 0;
    }

    public static class Command {
        public double linear_velocity;
        public double angular_velocity;

        public Command(double linear_velocity, double angular_velocity) {
            this.linear_velocity = linear_velocity;
            this.angular_velocity = angular_velocity;
        }
    }

    public Command update(RigidTransform2d robot_pose) {
        double distance_from_path = mPath.update(robot_pose.getTranslation());
        PathSegment.Sample lookahead_point = mPath.getLookaheadPoint(robot_pose.getTranslation(),
                distance_from_path + mFixedLookahead);
        Optional<Circle> circle = joinPath(robot_pose, lookahead_point.translation);
        // System.out.println("Pose is " + robot_pose + " and lookahead point is
        // " + lookahead_point.translation);

        if (circle.isPresent()) {
            // System.out
            // .println("Circle radius is " + circle.get().radius + " and center
            // point is " + circle.get().center);
            Translation2d robot_pose_inverse = robot_pose.getTranslation().inverse();
            Translation2d pose_to_circle = robot_pose_inverse.translateBy(circle.get().center);
            Translation2d pose_to_lookahead = robot_pose_inverse.translateBy(lookahead_point.translation);
            double cross_product = pose_to_circle.getX() * pose_to_lookahead.getY()
                    - pose_to_circle.getY() * pose_to_lookahead.getX();
            return new Command(lookahead_point.speed,
                    (cross_product >= 0 ? -1 : 1) * lookahead_point.speed / circle.get().radius);
        } else {
            return new Command(lookahead_point.speed, 0.0);
        }
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
