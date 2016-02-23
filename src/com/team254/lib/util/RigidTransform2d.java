package com.team254.lib.util;

/**
 * Represents a 2d pose (rigid transform) containing translational and
 * rotational elements.
 * 
 * This representation in SE(2) (see
 * https://en.wikipedia.org/wiki/Euclidean_group) has well defined behavior for
 * multiplication (transformBy) and inversion.
 * 
 * Inspired by Sophus (https://github.com/strasdat/Sophus/tree/master/sophus)
 * 
 * @author Jared
 */
public class RigidTransform2d implements Interpolable<RigidTransform2d> {
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

    // This currently does Riemannian interpolation. We could do twist
    // interpolation if necessary.
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
