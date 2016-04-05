package com.team254.lib.util;

public class AdaptivePurePursuitController {
    private static final double kEpsilon = 1E-9;
    
    double mFixedLookahead;

    public AdaptivePurePursuitController(double fixed_lookahead) {
        mFixedLookahead = fixed_lookahead;
    }

    public static class Circle {
        public double cx;
        public double cy;
        public double radius;

        public Circle(double cx, double cy, double radius) {
            this.cx = cx;
            this.cy = cy;
            this.radius = radius;
        }
    }

    public static Circle joinPath(RigidTransform2d robot_pose, Translation2d lookahead_point) {
        double x1 = robot_pose.getTranslation().getX();
        double y1 = robot_pose.getTranslation().getY();
        double x2 = lookahead_point.getX();
        double y2 = lookahead_point.getY();
        double dx = x1 - x2;
        double dy = y1 - y2;
        double cos = robot_pose.getRotation().cos();
        double sin = robot_pose.getRotation().sin();

        double cross_term = sin * dx + cos * dy;
        if (Math.abs(cross_term) < kEpsilon) {
            // Points are colinear
            return null;
        }

        return new Circle((sin * (x1 * x1 - x2 * x2 - dy * dy) + 2 * cos * x1 * dy) / (2 * cross_term),
                (-cos * (-y1 * y1 + x2 * x2 - dx * dx) + 2 * sin * y1 * dx) / (2 * cross_term),
                .5 * (dx * dx + dy * dy) / cross_term);
    }

}
