package com.team254.frc2016.auto.actions;

import com.team254.frc2016.subsystems.Superstructure;

public class StartAutoAimingAction implements Action {

    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private boolean mIsDone = false;

    @Override
    public void start() {
        mSuperstructure.setWantedState(Superstructure.WantedState.WANT_TO_AIM);
        mIsDone = true;
    }

    @Override
    public boolean isFinished() {
        return mIsDone;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

}
