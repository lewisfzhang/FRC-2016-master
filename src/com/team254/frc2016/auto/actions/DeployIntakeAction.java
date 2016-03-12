package com.team254.frc2016.auto.actions;

import com.team254.frc2016.subsystems.Intake;

public class DeployIntakeAction implements Action {

    private boolean mIsDone = false;
    private final Intake mIntake = Intake.getInstance();

    @Override
    public boolean isFinished() {
        return mIsDone;
    }

    @Override
    public void update() {}

    @Override
    public void done() {}

    @Override
    public void start() {
        mIntake.setDeploy(true);
        mIsDone = true;
    }
}
