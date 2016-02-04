package com.team254.frc2016.vision;

import com.team254.lib.util.Rotation2d;

public class TargetInfo {
    protected Rotation2d angle_;
    protected double distance_;

    public TargetInfo(double distance, Rotation2d angle) {
        distance_ = distance;
        angle_ = angle;
    }

    public Rotation2d getAngle() {
        return angle_;
    }

    public double getDistance() {
        return distance_;
    }
}