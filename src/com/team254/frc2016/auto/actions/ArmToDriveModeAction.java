package com.team254.frc2016.auto.actions;

import com.team254.frc2016.subsystems.UtilityArm;

/**
 * Moves the arm to drive mode then completes
 */
public class ArmToDriveModeAction implements Action {

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
        mUtilityArm.setWantedState(UtilityArm.WantedState.DRIVING);
        mIsDone = true;
    }
}
