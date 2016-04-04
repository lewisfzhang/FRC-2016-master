package com.team254.frc2016.auto.actions;

import com.team254.frc2016.subsystems.UtilityArm;

/**
 * Moves the arm to any mode then completes
 */
public class SetArmModeAction implements Action {

    private UtilityArm.WantedState mDesiredState;

    public SetArmModeAction(UtilityArm.WantedState state) {
        mDesiredState = state;
    }

    private final UtilityArm mUtilityArm = UtilityArm.getInstance();
    private boolean mIsDone = false;

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

    @Override
    public void start() {
        mUtilityArm.setWantedState(mDesiredState);
        mIsDone = true;
    }
}
