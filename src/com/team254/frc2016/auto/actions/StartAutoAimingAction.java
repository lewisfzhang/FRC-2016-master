package com.team254.frc2016.auto.actions;

import com.team254.frc2016.subsystems.Shooter;

public class StartAutoAimingAction implements Action {

    private final Shooter mShooter = Shooter.getInstance();
    private boolean mIsDone = false;

    @Override
    public void start() {
        mShooter.setWantedState(Shooter.WantedState.WANT_TO_AIM);
        mIsDone = true;
    }

    @Override
    public boolean isFinished() {
        return mIsDone;
    }

    @Override
    public void update() {}

    @Override
    public void done() {}

}
