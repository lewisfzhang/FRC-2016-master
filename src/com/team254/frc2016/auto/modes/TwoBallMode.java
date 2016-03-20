package com.team254.frc2016.auto.modes;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.*;
import com.team254.frc2016.subsystems.UtilityArm;

import java.util.ArrayList;
import java.util.Arrays;

public class TwoBallMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new WaitAction(1.0));
        runAction(new GetLowAction());
        runAction(new DeployIntakeAction());
        runAction(new WaitAction(3.0));
        runAction(
                new SeriesAction(Arrays.asList(
                        new DriveStraightAction(-150, -45),
                        new WaitAction(.01),
                        new DriveStraightAction(0.1, 0)
                ))
        );
    }
}
