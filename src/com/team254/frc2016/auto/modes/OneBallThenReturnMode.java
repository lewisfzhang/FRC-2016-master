package com.team254.frc2016.auto.modes;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.DriveStraightAction;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Shooter;

/**
 * The typical one ball auto mode.
 */
public class OneBallThenReturnMode extends AutoModeBase {

    private final Drive mDrive;
    private final OneBallMode mOneBallMode;

    public OneBallThenReturnMode(Drive drive, Shooter shooter) {
        mDrive = drive;
        mOneBallMode = new OneBallMode(shooter);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Starting One Ball With Return Mode");
        mOneBallMode.doOneBallRoutine();
        runAction(new DriveStraightAction(
                -(mDrive.getLeftDistanceInches() + mDrive.getRightDistanceInches()) / 2.0 + 16.0,
                -45));
        System.out.println("Ending");
    }
}
