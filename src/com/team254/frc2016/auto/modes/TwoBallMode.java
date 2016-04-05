package com.team254.frc2016.auto.modes;

import java.util.Arrays;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.*;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Intake;
import com.team254.frc2016.subsystems.Superstructure;
import com.team254.frc2016.subsystems.Superstructure.WantedState;
import com.team254.frc2016.subsystems.UtilityArm;
import com.team254.lib.util.Rotation2d;

public class TwoBallMode extends AutoModeBase {
    Superstructure mSuperstructure = Superstructure.getInstance();
    Drive mDrive = Drive.getInstance();
    double mDistanceToDrive;

    public TwoBallMode(double distance_to_drive) {
        mDistanceToDrive = distance_to_drive;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        mSuperstructure.setWantsToRunIntake();
        mSuperstructure.deployIntake();
        runAction(new ParallelAction(Arrays.asList(new GetLowAction(), new WaitAction(1))));
        mSuperstructure.setWantsToStopIntake();

        runAction(new ParallelAction(
                Arrays.asList(new DriveStraightAction(mDistanceToDrive, AutoModeUtils.TWO_BALL_FORWARD_DRIVE_VELOCITY),
                        new SeriesAction(Arrays.asList(new WaitForDistanceAction(AutoModeUtils.DISTANCE_TO_POP_HOOD),
                                new StartAutoAimingAction(), new PointTurretAction(Rotation2d.fromDegrees(-45.0)))))));
        runAction(new ShootWhenReadyAction());
        runAction(new WaitAction(1.0));
        mSuperstructure.setWantedState(WantedState.WANT_TO_STOW);
        mSuperstructure.setWantsToRunIntake();
        runAction(new DriveStraightAction(-mDistanceToDrive + 6.0, -AutoModeUtils.TWO_BALL_FORWARD_DRIVE_VELOCITY));
        mDrive.setVelocitySetpoint(0, 0);

        runAction(new ParallelAction(Arrays.asList(
                new DriveStraightAction(mDistanceToDrive - 6.0, AutoModeUtils.TWO_BALL_FORWARD_DRIVE_VELOCITY),
                new SeriesAction(Arrays.asList(new WaitForDistanceAction(AutoModeUtils.DISTANCE_TO_POP_HOOD),
                        new StartAutoAimingAction(), new PointTurretAction(Rotation2d.fromDegrees(-45.0)))))));
        runAction(new ShootWhenReadyAction());
        runAction(new WaitAction(1.0));
        mSuperstructure.setWantedState(WantedState.WANT_TO_STOW);
        mSuperstructure.setWantsToStopIntake();

    }
}
