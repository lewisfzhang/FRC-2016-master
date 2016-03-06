package com.team254.frc2016.auto.actions;

import com.team254.frc2016.subsystems.UtilityArm;

/**
 * Action for putting the utility arm and intake low
 */
public class GetLowAction implements Action {

    private final UtilityArm mUtilityArm = UtilityArm.getInstance();

    @Override
    public boolean isFinished() {
        return mUtilityArm.isSafeToDriveThroughPortcullis();
    }

    @Override
    public void start() {
        mUtilityArm.setWantedState(UtilityArm.WantedState.PORTCULLIS);
    }

    @Override
    public void update() {}

    @Override
    public void done() {}
}
