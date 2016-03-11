package com.team254.frc2016.auto.modes;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.*;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Shooter;
import com.team254.frc2016.subsystems.UtilityArm;

import java.util.Arrays;

/**
 * Get's into portcullis/low bar position before driving through the defense.
 */
public class GetLowOneBallMode extends AutoModeBase {

    private final boolean mIsBadBall;
    private final boolean mShouldDriveBack;
    private final double mDistanceToDrive;
    private final Shooter mShooter = Shooter.getInstance();
    private final Drive mDrive = Drive.getInstance();

    public GetLowOneBallMode(boolean isBadBall, boolean shouldDriveBack, double distanceToDrive) {
        mIsBadBall = isBadBall;
        mShouldDriveBack = shouldDriveBack;
        mDistanceToDrive = distanceToDrive;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        mShooter.setIsBadBall(mIsBadBall);

        runAction(new GetLowAction());
        runAction(new WaitAction(3.5));

        runAction(new ParallelAction(Arrays.asList(
                new DriveStraightAction(mDistanceToDrive, AutoModeUtils.FORWARD_DRIVE_VELOCITY),
                new SeriesAction(Arrays.asList(
                        new WaitForDistanceAction(AutoModeUtils.DISTANCE_TO_POP_HOOD),
                        new StartAutoAimingAction()
                ))
        )));

        runAction(new WaitAction(1));
        runAction(new ShootWhenReadyAction());
        runAction(new WaitAction(0.75));
        runAction(new SetArmModeAction(UtilityArm.WantedState.DRIVING));

        if (mShouldDriveBack) {
            runAction(AutoModeUtils.makeDriveBackAction(mDrive));
            mDrive.setVelocitySetpoint(0, 0);
        }
    }
}
