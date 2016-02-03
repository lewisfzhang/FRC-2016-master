package com.team254.frc2016.vision;

public class TargetInfo {
    protected double theta;
    protected double distance;

    TargetInfo(double theta, double distance) {
        this.theta = theta;
        this.distance = distance;
    }

    public double getTheta() {
        return theta;
    }

    public double getDistance() {
        return distance;
    }
}