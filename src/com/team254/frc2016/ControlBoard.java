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
        return mTurnStick.getRawButton(2);
    }

    public boolean getLowGear() {
        return mThrottleStick.getRawButton(2);
    }

    public boolean getFireButton() {
        return mThrottleStick.getRawButton(1);
    }

    // OPERATOR CONTROLS
    public boolean getAutoHangEnabled() {
        return mButtonBoard.getRawAxis(3) > 0.1;
    }

    public boolean getHang() {
        return mButtonBoard.getRawButton(7);
    }

    public boolean getDeployHanger() {
        return mButtonBoard.getRawButton(8);
    }

    public double getTurretManual() {
        if (mButtonBoard.getRawButton(11)) {
            return 1.0;
        } else if (mButtonBoard.getRawButton(12)) {
            return -1.0;
        } else {
            return 0.0;
        }
    }

    public boolean getAutoAim() {
        return mButtonBoard.getRawButton(10);
    }

    public boolean getBatterShot() {
        return mButtonBoard.getRawButton(9);
    }

    public boolean getBailButton() {
        return mButtonBoard.getRawButton(3);
    }

    public boolean getButton4() {
        return mButtonBoard.getRawButton(4);
    }

    public boolean getButton5() {
        return mButtonBoard.getRawButton(5);
    }

    public boolean getButton6() {
        return mButtonBoard.getRawButton(6);
    }

    public boolean getIntakeButton() {
        return mButtonBoard.getRawAxis(2) < -0.1;
    }

    public boolean getStowIntakeButton() {
        return mButtonBoard.getRawAxis(1) < -0.1;
    }

    public boolean getExhaustButton() {
        return mButtonBoard.getRawAxis(0) < -0.1;
    }
}
