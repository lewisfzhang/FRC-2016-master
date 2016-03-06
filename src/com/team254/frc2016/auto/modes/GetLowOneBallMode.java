package com.team254.frc2016.auto.modes;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.*;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Shooter;

import java.util.Arrays;

/**
 * Get's into portcullis/low bar position before driving through the defense.
 */
public class GetLowOneBallMode extends AutoModeBase {

    private final boolean mIsBadBall;
    private final boolean mShouldDriveBack;
    private final Shooter mShooter = Shooter.getInstance();
    private final Drive mDrive = Drive.getInstance();

    public GetLowOneBallMode(boolean isBadBall, boolean shouldDriveBack) {
        mIsBadBall = isBadBall;
        mShouldDriveBack = shouldDriveBack;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        mShooter.setIsBadBall(mIsBadBall);
        runAction(
                new ParallelAction(
                        Arrays.<Action>asList(
                                new DriveThenAimAction(125, 100, 270, 45),
                                new GetLowAction())));
        runAction(new WaitAction(1));
        mShooter.setWantsToFireNow();
        runAction(new WaitAction(0.75));

        if (mShouldDriveBack) {
            runAction(
                    new DriveStraightAction(
                            -(mDrive.getLeftDistanceInches() + mDrive.getRightDistanceInches())
                                    / 2.0 + 16.0,
                            -45));
        }
    }
}
