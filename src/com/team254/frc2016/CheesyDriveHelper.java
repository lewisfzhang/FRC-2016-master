package com.team254.frc2016;

import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Util;
/**
 * Created by ryanjohnson on 1/30/16.
 */

public class CheesyDriveHelper {

    double quickStopAccumulator;
    private static final double throttleDeadband = 0.02;
    private static final double wheelDeadband = 0.02;
    private DriveSignal signal = new DriveSignal(0, 0);

    public DriveSignal cheesyDrive(double throttle, double wheel, boolean isQuickTurn,
                                   boolean isHighGear) {

        // Apply deadbands to joystick inputs
        wheel = handleDeadband(wheel, wheelDeadband);
        throttle = handleDeadband(throttle, throttleDeadband);

        // Apply a sin function that's scaled to make it feel better.
        if (isHighGear) {
            wheel = applyNonLinearity(wheel, 0.6, 2);
        } else {
            wheel = applyNonLinearity(wheel, 0.5, 3);
        }

        double overPower;

        double angularPower;
        double linearPower = throttle; // alias for throttle


        // Quickturn!
        if (isQuickTurn) {
            if (Math.abs(linearPower) < 0.2) {
                double alpha = 0.1;
                // quickStopAccumulator approaches wheel*5
                quickStopAccumulator = (1 - alpha) * quickStopAccumulator
                        + alpha * Util.limit(wheel, 1.0) * 5;
            }
            overPower = 1.0;
            angularPower = wheel;
        } else {
            overPower = 0.0;
            double turnSensitivity = isHighGear? 1.0 : 0.85;
            angularPower = Math.abs(throttle)*wheel*turnSensitivity - quickStopAccumulator;
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
        double rightPwm = linearPower - angularPower;
        double leftPwm = linearPower + angularPower;
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

    // Sinusoidal Non-linearity
    private double applyNonLinearity(Double wheel, Double constant, int times) {
        for (int i = 0; i < times; i++) {
            wheel = Math.sin(Math.PI / 2.0 * constant * wheel)
                    / Math.sin(Math.PI / 2.0 * constant);
        }
        return wheel;
    }

    public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }
}
