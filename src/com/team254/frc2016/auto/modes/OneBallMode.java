package com.team254.frc2016.auto.modes;

import com.team254.frc2016.RobotState;
import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.Action;
import com.team254.frc2016.auto.actions.DriveStraightAction;
import com.team254.frc2016.auto.actions.DriveUntilInRangeAction;
import com.team254.frc2016.auto.actions.WaitAction;
import com.team254.frc2016.subsystems.Drive;
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
    private Rotation2d mFoundTurretRotation = Rotation2d.fromDegrees(0);
    RobotState mRobotState = RobotState.getInstance();

    public OneBallMode(Drive drive, Shooter shooter) {
        mDrive = drive;
        mShooter = shooter;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Starting");
        runAction(new DriveStraightAction(190, 30));

        System.out.println("Finding targets");
        mShooter.setWantedState(Shooter.WantedState.WANT_TO_AIM);
        runAction(new WaitAction(1.5)); // TODO: Make this better, sometimes it
                                        // misses the target with a small delay

        runAction(mPanForTargetAction);

        runAction(new WaitAction(.25));
        System.out.println("Turret angle found: " + mFoundTurretRotation.getDegrees());

        runAction(new DriveUntilInRangeAction(35, 88, 120));

        runAction(new WaitAction(1));
        mShooter.setWantsToFireNow();
        runAction(new WaitAction(1));
        mShooter.setWantedState(Shooter.WantedState.WANT_TO_STOW);
        System.out.println("Ending");
    }

    private final Action mPanForTargetAction = new Action() {

        private double mStartTime;
        private boolean mFoundTarget = false;
        private boolean mSign = true;

        @Override
        public void start() {
            mShooter.setWantedState(Shooter.WantedState.WANT_TO_AIM);
            mStartTime = Timer.getFPGATimestamp();
        }

        @Override
        public void update() {
            // TODO: scan left/right if the target can't be found
            List<ShooterAimingParameters> list = mShooter.getCachedAimingParams();
            mFoundTarget = !list.isEmpty();

            if (mFoundTarget) {
                mFoundTurretRotation = list.get(0).getTurretAngle();
                return;
            }

            double output = mSign ? .5 : -.5;

            System.out.println("time: " + (Timer.getFPGATimestamp() - mStartTime) + " l: " + list.size() + " turret: "
                    + mRobotState.getLatestTurretRotation().getValue().getDegrees());
            if (mSign && mRobotState.getLatestTurretRotation().getValue().getDegrees() > 45) {
                mSign = false;
            }
            if (!mSign && mRobotState.getLatestTurretRotation().getValue().getDegrees() < -45) {
                mSign = true;
            }
            mShooter.setTurretManualScanOutput(output);
        }

        @Override
        public boolean isFinished() {
            return mFoundTarget;
        }

        @Override
        public void done() {
            mShooter.setTurretManualScanOutput(0);
        }
    };
}
