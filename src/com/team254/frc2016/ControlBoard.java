package com.team254.frc2016;

import edu.wpi.first.wpilibj.Joystick;

public class ControlBoard {
    private static ControlBoard mInstance = new ControlBoard();

    public static ControlBoard getInstance() {
        return mInstance;
    }

    private final Joystick mThrottleStick;
    private final Joystick mTurnStick;
    private final Joystick mButtonBoard;

    private ControlBoard() {
        mThrottleStick = new Joystick(0);
        mTurnStick = new Joystick(1);
        mButtonBoard = new Joystick(2);
    }

    // DRIVER CONTROLS
    public double getThrottle() {
        return -mThrottleStick.getY();
    }

    public double getTurn() {
        return mTurnStick.getX();
    }

    public boolean getQuickTurn() {
        return mTurnStick.getRawButton(1);
    }

    public boolean getBaseLock() {
        return mThrottleStick.getRawButton(1);
    }

    public boolean getLowGear() {
        return mThrottleStick.getRawButton(0);
    }

    public boolean getAutoFireButton() {
        return mTurnStick.getRawButton(0);
    }

    // OPERATOR CONTROLS
}
