package com.team254.frc2016.auto.modes;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.*;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.UtilityArm;

import java.util.Arrays;

public class ShovelTheFriesMode extends AutoModeBase {

    private double mDistanceToCDF, mDistanceToDrive;
    private final Drive mDrive = Drive.getInstance();

    public ShovelTheFriesMode(double distanceToShootAt) {
        mDistanceToCDF = 43.5;
        mDistanceToDrive = distanceToShootAt - mDistanceToCDF;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new DriveStraightAction(mDistanceToCDF, AutoModeUtils.FORWARD_DRIVE_VELOCITY));
        mDrive.setVelocitySetpoint(0, 0);
        runAction(new WaitAction(0.5));
        runAction(new GetLowAction());
        runAction(new WaitAction(0.5));
        runAction(new ParallelAction(Arrays.asList(
                new DriveStraightAction(mDistanceToDrive, AutoModeUtils.FORWARD_DRIVE_VELOCITY),
                new SeriesAction(Arrays.asList(
                        new WaitForDistanceAction(AutoModeUtils.DISTANCE_TO_POP_HOOD - mDistanceToCDF),
                        new StartAutoAimingAction()
                ))
        )));

        runAction(new WaitAction(1));
        runAction(new ShootWhenReadyAction());
        runAction(new WaitAction(0.75));
        runAction(new SetArmModeAction(UtilityArm.WantedState.DRIVING));

    }
}
