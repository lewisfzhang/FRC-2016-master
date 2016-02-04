package com.team254.frc2016;

import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Util;

/**
 * Created by ryanjohnson on 1/30/16.
 */

public class CheesyDriveHelper {

    double quickStopAccumulator;
    private static final double kThrottleDeadband = 0.02;
    private static final double kWheelDeadband = 0.02;
    private static final double kTurnSensitivity = 1.0;
    private DriveSignal signal = new DriveSignal(0, 0);

    public DriveSignal cheesyDrive(double throttle, double wheel, boolean isQuickTurn) {

        // Apply deadbands to joystick inputs
        wheel = handleDeadband(wheel, kWheelDeadband);
        throttle = handleDeadband(throttle, kThrottleDeadband);

        double overPower;

        double angularPower;

        // Quickturn!
        if (isQuickTurn) {
            if (Math.abs(throttle) < 0.2) {
                double alpha = 0.1;
                // quickStopAccumulator approaches wheel*2
                quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha * Util.limit(wheel, 1.0) * 2;
            }
            overPower = 1.0;
            angularPower = wheel;
        } else {
            overPower = 0.0;
            angularPower = Math.abs(throttle) * wheel * kTurnSensitivity - quickStopAccumulator;
            // quickStopAccumulator -> 0 (gradually, by 1 per step)
            if (quickStopAccumulator > 1) {
                quickStopAccumulator -= 1;
            } else if (quickStopAccumulator < -1) {
                quickStopAccumulator += 1;
            } else {
                quickStopAccumulator = 0.0;
            }
        }

        // Calculate final L/R output values
        double rightPwm = throttle - angularPower;
        double leftPwm = throttle + angularPower;
        if (leftPwm > 1.0) {
            rightPwm -= overPower * (leftPwm - 1.0);
            leftPwm = 1.0;
        } else if (rightPwm > 1.0) {
            leftPwm -= overPower * (rightPwm - 1.0);
            rightPwm = 1.0;
        } else if (leftPwm < -1.0) {
            rightPwm += overPower * (-1.0 - leftPwm);
            leftPwm = -1.0;
        } else if (rightPwm < -1.0) {
            leftPwm += overPower * (-1.0 - rightPwm);
            rightPwm = -1.0;
        }
        signal.rightMotor = rightPwm;
        signal.leftMotor = leftPwm;
        return signal;
    }

    public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }
}