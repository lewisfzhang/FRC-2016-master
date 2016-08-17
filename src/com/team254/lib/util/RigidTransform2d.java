package com.team254.lib.util;

/**
 * Represents a 2d pose (rigid transform) containing translational and
 * rotational elements.
 * 
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 */
public class RigidTransform2d implements Interpolable<RigidTransform2d> {
    private final static double kEps = 1E-9;

    // A tangent space velocity (e.g. movement along an arc at a constant local
    // velocity)
    public static class Delta {
        public final double dx;
        public final double dy;
        public final double dtheta;

        public Delta(double dx, double dy, double dtheta) {
            this.dx = dx;
            this.dy = dy;
            this.dtheta = dtheta;
        }
    }

    protected Translation2d translation_;
    protected Rotation2d rotation_;

    public RigidTransform2d() {
        translation_ = new Translation2d();
        rotation_ = new Rotation2d();
    }

    public RigidTransform2d(Translation2d translation, Rotation2d rotation) {
        translation_ = translation;
        rotation_ = rotation;
    }

    public RigidTransform2d(RigidTransform2d other) {
        translation_ = new Translation2d(other.translation_);
        rotation_ = new Rotation2d(other.rotation_);
    }

    public static RigidTransform2d fromTranslation(Translation2d translation) {
        return new RigidTransform2d(translation, new Rotation2d());
    }

    public static RigidTransform2d fromRotation(Rotation2d rotation) {
        return new RigidTransform2d(new Translation2d(), rotation);
    }

    /** SE(2) exponential map
     *  https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
     */
    public static RigidTransform2d fromVelocity(Delta delta) {
        double sin_theta = Math.sin(delta.dtheta);
        double cos_theta = Math.cos(delta.dtheta);
        double s, c;
        if (Math.abs(delta.dtheta) < kEps) {
            s = 1.0 - 1.0 / 6.0 * delta.dtheta * delta.dtheta;
            c = .5 * delta.dtheta;
        } else {
            s = sin_theta / delta.dtheta;
            c = (1.0 - cos_theta) / delta.dtheta;
        }
        return new RigidTransform2d(new Translation2d(delta.dx * s - delta.dy * c, delta.dx * c + delta.dy * s),
                new Rotation2d(cos_theta, sin_theta, false));
    }

    public Translation2d getTranslation() {
        return translation_;
    }

    public void setTranslation(Translation2d translation) {
        translation_ = translation;
    }

    public Rotation2d getRotation() {
        return rotation_;
    }

    public void setRotation(Rotation2d rotation) {
        rotation_ = rotation;
    }

    public RigidTransform2d transformBy(RigidTransform2d other) {
        return new RigidTransform2d(translation_.translateBy(other.translation_.rotateBy(rotation_)),
                rotation_.rotateBy(other.rotation_));
    }

    public RigidTransform2d inverse() {
        Rotation2d rotation_inverted = rotation_.inverse();
        return new RigidTransform2d(translation_.inverse().rotateBy(rotation_inverted), rotation_inverted);
    }

    /** This does Riemannian interpolation and can do twist interpolation if necessary. */
    @Override
    public RigidTransform2d interpolate(RigidTransform2d other, double x) {
        if (x <= 0) {
            return new RigidTransform2d(this);
        } else if (x >= 1) {
            return new RigidTransform2d(other);
        }
        return new RigidTransform2d(translation_.interpolate(other.translation_, x),
                rotation_.interpolate(other.rotation_, x));
    }

    @Override
    public String toString() {
        return "T:" + translation_.toString() + ", R:" + rotation_.toString();
    }
}
