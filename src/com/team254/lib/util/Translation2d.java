package com.team254.lib.util;

import java.text.DecimalFormat;

/**
 * A translation in a 2d coordinate frame.
 * 
 * @author Jared
 */
public class Translation2d implements Interpolable<Translation2d> {
    protected double x_;
    protected double y_;

    public Translation2d() {
        x_ = 0;
        y_ = 0;
    }

    public Translation2d(double x, double y) {
        x_ = x;
        y_ = y;
    }

    public Translation2d(Translation2d other) {
        x_ = other.x_;
        y_ = other.y_;
    }

    public double norm() {
        return Math.hypot(x_, y_);
    }

    public double getX() {
        return x_;
    }

    public double getY() {
        return y_;
    }

    public void setX(double x) {
        x_ = x;
    }

    public void setY(double y) {
        y_ = y;
    }

    public Translation2d translateBy(Translation2d other) {
        return new Translation2d(x_ + other.x_, y_ + other.y_);
    }

    public Translation2d rotateBy(Rotation2d rotation) {
        return new Translation2d(x_ * rotation.cos() - y_ * rotation.sin(), x_ * rotation.sin() + y_ * rotation.cos());
    }

    public Translation2d inverse() {
        return new Translation2d(-x_, -y_);
    }

    public Translation2d interpolate(Number xValue, Number otherXValue, Translation2d otherYValue,
            Number interpolatedXValue) {
        if (xValue.doubleValue() == otherXValue.doubleValue()) {
            return new Translation2d(this);
        }
        double interp = (interpolatedXValue.doubleValue() - xValue.doubleValue())
                / (otherXValue.doubleValue() - xValue.doubleValue());
        return new Translation2d(interp * (otherYValue.x_ - x_) + x_, interp * (otherYValue.y_ - y_) + y_);
    }

    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(x_) + "," + fmt.format(y_) + ")";
    }
}
