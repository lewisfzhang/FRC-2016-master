package com.team254.frc2016.auto.modes;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.*;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Shooter;
import com.team254.frc2016.subsystems.UtilityArm;

import java.util.Arrays;

/**
 * Go over the defenses in starting config
 */
public class StayHighOneBall extends AutoModeBase {

    private final Drive mDrive = Drive.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final boolean mIsBadBall;
    private final boolean mShouldDriveBack;
    private final double mDistanceToDrive;

    // TODO: validate this distance
    public static final double DISTANCE_TO_DROP_ARM = 85;

    public StayHighOneBall(boolean isBadBall, boolean shouldDriveBack, double distanceToDrive) {
        mIsBadBall = isBadBall;
        mShouldDriveBack = shouldDriveBack;
        mDistanceToDrive = distanceToDrive;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        mShooter.setIsBadBall(mIsBadBall);

        runAction(new ParallelAction(Arrays.asList(
                new DriveStraightAction(mDistanceToDrive, AutoModeUtils.FORWARD_DRIVE_VELOCITY),
                new SeriesAction(Arrays.asList(
                        new WaitForDistanceAction(DISTANCE_TO_DROP_ARM),
                        new SetArmModeAction(UtilityArm.WantedState.PORTCULLIS),
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
        }
    }
}
