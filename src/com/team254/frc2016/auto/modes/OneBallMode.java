package com.team254.frc2016.auto.modes;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.DriveThenAimAction;
import com.team254.frc2016.auto.actions.ShootWhenReadyAction;
import com.team254.frc2016.auto.actions.WaitAction;
import com.team254.frc2016.subsystems.Shooter;

/**
 * The typical one ball auto mode.
 */
public class OneBallMode extends AutoModeBase {

    private final Shooter mShooter;
    private final boolean mIsBadBall;

    public OneBallMode(Shooter shooter, boolean isBadBall) {
        mShooter = shooter;
        mIsBadBall = isBadBall;
    }

    public void doOneBallRoutine() throws AutoModeEndedException {
        mShooter.setIsBadBall(mIsBadBall);
        runAction(new DriveThenAimAction(125, 100, 270, 45));
        runAction(new WaitAction(2));
        runAction(new ShootWhenReadyAction());
        runAction(new WaitAction(0.75));
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Starting One Ball Mode");
        doOneBallRoutine();
        System.out.println("Ending");
    }
}
