package com.team254.frc2016.auto.actions;

import com.team254.frc2016.subsystems.Drive;
import com.team254.lib.util.Rotation2d;
import edu.wpi.first.wpilibj.InterruptHandlerFunction;

public class DriveUntilLineAction implements Action {

    private double mVelocity = -16.0;
    private double mHeading = 0;
    private Drive mDrive = Drive.getInstance();

    @Override
    public boolean isFinished() {
        return mDrive.getLineSensorTriggered();
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {
        mDrive.setBaseLockOn();
    }

    @Override
    public void start() {
        mDrive.setHighGear(false);
        mDrive.setVelocityHeadingSetpoint(mVelocity, Rotation2d.fromDegrees(mHeading));
    }

    private double getCurrentDistance() {
        return (mDrive.getLeftDistanceInches() + mDrive.getRightDistanceInches()) / 2;
    }
}
