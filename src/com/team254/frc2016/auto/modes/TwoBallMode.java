package com.team254.frc2016.auto.modes;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.*;
import com.team254.frc2016.subsystems.Intake;

public class TwoBallMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new DriveUntilLineAction());
        Intake.getInstance().setDeploy(true);
        Intake.getInstance().setIntakeRoller(1.0);
        runAction(new WaitAction(1.5));
        Intake.getInstance().setIntakeRoller(0.0);
        runAction(new DriveStraightAction(20, 20));
    }
}
