package com.team254.frc2016.auto.modes;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.*;
import com.team254.frc2016.subsystems.Intake;
import com.team254.frc2016.subsystems.Superstructure;

public class TwoBallMode extends AutoModeBase {
    Superstructure mSuperstructure = Superstructure.getInstance();

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new DriveUntilLineAction());
        mSuperstructure.deployIntake();
        mSuperstructure.setWantsToRunIntake();
        runAction(new WaitAction(1.5));
        mSuperstructure.setWantsToStopIntake();
        runAction(new DriveStraightAction(20, 20));
    }
}
