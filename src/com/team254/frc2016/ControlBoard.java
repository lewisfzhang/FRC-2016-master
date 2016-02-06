package com.team254.frc2016;

import edu.wpi.first.wpilibj.Joystick;

public class ControlBoard {
    private static ControlBoard ourInstance = new ControlBoard();

    public static ControlBoard getInstance() {
        return ourInstance;
    }

    private final Joystick throttleStick;
    private final Joystick turnStick;

    private ControlBoard() {
        throttleStick = new Joystick(0);
        turnStick = new Joystick(1);
    }

    public double getThrottle() {
        return -throttleStick.getY();
    }

    public double getTurn() {
        return turnStick.getX();
    }

    public boolean getQuickTurn() {
        return turnStick.getRawButton(1);
    }
    
    public boolean getBaseLock() {
        return throttleStick.getRawButton(1);
    }
}
