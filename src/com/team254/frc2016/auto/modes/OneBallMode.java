package com.team254.frc2016.auto.modes;

import com.team254.frc2016.RobotState;
import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.Action;
import com.team254.frc2016.auto.actions.DriveStraightAction;
import com.team254.frc2016.auto.actions.DriveThenAimAction;
import com.team254.frc2016.auto.actions.DriveUntilInRangeAction;
import com.team254.frc2016.auto.actions.WaitAction;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Intake;
import com.team254.frc2016.subsystems.Shooter;
import com.team254.frc2016.subsystems.ShooterAimingParameters;
import com.team254.lib.util.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.List;

/**
 * The typical one ball auto mode.
 */
public class OneBallMode extends AutoModeBase {

    private final Drive mDrive;
    private final Shooter mShooter;
    private final Intake mIntake = Intake.getInstance();
    RobotState mRobotState = RobotState.getInstance();

    public OneBallMode(Drive drive, Shooter shooter) {
        mShooter = shooter;
        mDrive = drive;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Starting");
        runAction(new DriveThenAimAction(125, 100, 270, 45));
        runAction(new WaitAction(1));
        mShooter.setWantsToFireNow();
        runAction(new WaitAction(0.75));
        mShooter.setWantedState(Shooter.WantedState.WANT_TO_STOW);
        runAction(new DriveStraightAction(
                -(mDrive.getLeftDistanceInches() + mDrive.getRightDistanceInches()) / 2.0 + 16.0, -45));
        System.out.println("Ending");
    }
}
