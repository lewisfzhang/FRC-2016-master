package com.team254.frc2016.auto.actions;

import com.team254.frc2016.subsystems.Shooter;

public class ShootWhenReadyAction implements Action {

    private final Shooter mShooter = Shooter.getInstance();

    private int mNumShotsFiredAtStart;

    @Override
    public boolean isFinished() {
        return mShooter.getNumShotsFired() > mNumShotsFiredAtStart;
    }

    @Override
    public void start() {
        mNumShotsFiredAtStart = mShooter.getNumShotsFired();
        mShooter.setWantsToFireWhenReady();
    }

    @Override
    public void update() {}

    @Override
    public void done() {
        mShooter.setWantedState(Shooter.WantedState.WANT_TO_STOW);
    }
}
