package com.team254.frc2016.auto.modes;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.Action;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Shooter;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Rotation2d;

/**
 * The typical one ball auto mode.
 */
public class OneBallMode extends AutoModeBase {

    private final Drive mDrive;
    private final Shooter mShooter;

    public OneBallMode(Drive drive, Shooter shooter) {
        mDrive = drive;
        mShooter = shooter;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(mDriveStraightAction);
        runAction(mAutoShootAction);
    }

    // TODO: these into the .actions package
    private final Action mDriveStraightAction = new Action() {

        private double startingDistance;

        @Override
        public void start() {
            startingDistance = getCurrentDistance();
            mDrive.setHighGear(false);
            mDrive.setVelocityHeadingSetpoint(30, Rotation2d.fromDegrees(0));
        }

        @Override
        public void update() {
        }

        @Override
        public boolean isFinished() {
            return getCurrentDistance() - startingDistance >= 100;
        }

        @Override
        public void done() {
            mDrive.setOpenLoop(DriveSignal.NEUTRAL);
        }

        private double getCurrentDistance() {
            return (mDrive.getLeftDistanceInches() + mDrive.getRightDistanceInches()) / 2;
        }
    };

    private final Action mAutoShootAction = new Action() {

        private long mStartTime;

        @Override
        public void start() {
            mShooter.setWantedState(Shooter.WantedState.WANT_TO_AIM);
            mStartTime = System.currentTimeMillis();
        }

        @Override
        public void update() {
            // TODO: scan left/right if the target can't be found
        }

        @Override
        public boolean isFinished() {
            return System.currentTimeMillis() - mStartTime > 2000;
        }

        @Override
        public void done() {
            mShooter.setWantedState(Shooter.WantedState.WANT_TO_STOW);
        }
    };
}
