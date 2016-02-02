package com.team254.lib.util;

/**
 * A rotation in a 2d coordinate frame represented a point on the unit circle
 * (cosine and sine).
 * 
 * This representation in SO(2) (see https://en.wikipedia.org/wiki/Circle_group)
 * has well defined behavior for multiplication (rotateBy) and inversion.
 * 
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 * 
 * @author Jared
 */
public class Rotation2d {
    protected static final double kEpsilon = 1E-9;

    protected double cos_angle_;
    protected double sin_angle_;

    public Rotation2d() {
        this(1, 0, false);
    }

    protected Rotation2d(double cosine, double sine, boolean normalize) {
        cos_angle_ = cosine;
        sin_angle_ = sine;
        if (normalize) {
            normalize();
        }
    }

    public static Rotation2d fromRadians(double angle_radians) {
        return new Rotation2d(Math.cos(angle_radians), Math.sin(angle_radians), false);
    }

    public static Rotation2d fromDegrees(double angle_degrees) {
        return fromRadians(Math.toRadians(angle_degrees));
    }

    public void normalize() {
        double magnitude = Math.hypot(cos_angle_, sin_angle_);
        if (magnitude > kEpsilon) {
            sin_angle_ /= magnitude;
            cos_angle_ /= magnitude;
        } else {
            sin_angle_ = 0;
            cos_angle_ = 1;
        }
    }

    public double cos() {
        return cos_angle_;
    }

    public double sin() {
        return sin_angle_;
    }

    public double tan() {
        return sin_angle_ / cos_angle_;
    }

    public double getRadians() {
        return Math.atan2(sin_angle_, cos_angle_);
    }

    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }

    public Rotation2d rotateBy(Rotation2d other) {
        return new Rotation2d(cos_angle_ * other.cos_angle_ - sin_angle_ * other.sin_angle_,
                cos_angle_ * other.sin_angle_ + sin_angle_ * other.cos_angle_, false);
    }

    public Rotation2d inverse() {
        return new Rotation2d(cos_angle_, -sin_angle_, false);
    }
}
