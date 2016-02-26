package com.team254.frc2016.subsystems;

import com.team254.lib.util.Rotation2d;

public class ShooterAimingParameters {
    double range;
    Rotation2d turret_angle;
    int track_id;

    public ShooterAimingParameters(double range, Rotation2d turret_angle, int track_id) {
        this.range = range;
        this.turret_angle = turret_angle;
        this.track_id = track_id;
    }

    public double getRange() {
        return range;
    }

    public Rotation2d getTurretAngle() {
        return turret_angle;
    }

    public int getTrackid() {
        return track_id;
    }
}
