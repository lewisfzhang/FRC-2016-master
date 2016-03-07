package com.team254.lib.util;

/**
 * Created by ryanjohnson on 1/30/16.
 */
public class DriveSignal {
    public double leftMotor;
    public double rightMotor;
    public boolean breakMode;

    public DriveSignal(double left, double right) {
        this(left, right, false);
    }

    public DriveSignal(double left, double right, boolean breakMode) {
        this.leftMotor = left;
        this.rightMotor = right;
        this.breakMode = breakMode;
    }

    public static DriveSignal NEUTRAL = new DriveSignal(0, 0);
    public static DriveSignal BREAK = new DriveSignal(0, 0, true);

    @Override
    public String toString() {
        return "L: " + leftMotor + ", R: " + rightMotor;
    }
}