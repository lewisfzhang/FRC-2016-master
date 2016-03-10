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

    public final double DISTANCE_TO_DROP_ARM = 35;

    public StayHighOneBall(boolean isBadBall, boolean shouldDriveBack) {
        mIsBadBall = isBadBall;
        mShouldDriveBack = shouldDriveBack;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        mShooter.setIsBadBall(mIsBadBall);
        runAction(new ParallelAction(Arrays.asList(DriveThenAimAction.makeForCommonConsts(),
                new SeriesAction(Arrays.asList(
                        new WaitForDistanceAction(DISTANCE_TO_DROP_ARM),
                        new SetArmModeAction(UtilityArm.WantedState.PORTCULLIS)
                ))
        )));
        runAction(new WaitAction(1));
        runAction(new ShootWhenReadyAction());
        runAction(new WaitAction(0.75));
        runAction(new ArmToDriveModeAction());
        if (mShouldDriveBack) {
            runAction(new DriveStraightAction(
                    -(mDrive.getLeftDistanceInches() + mDrive.getRightDistanceInches()) / 2.0 + 16.0, -45));
        }
    }
}
