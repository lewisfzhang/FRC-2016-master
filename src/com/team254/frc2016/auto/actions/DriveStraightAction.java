package com.team254.frc2016.auto.actions;

import com.team254.frc2016.subsystems.Drive;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Rotation2d;

public class DriveStraightAction implements Action {

    private double startingDistance;
    private double mWantedDistance;
    private double mVelocity;
    private double mHeading;
    private Drive mDrive = Drive.getInstance();

    public DriveStraightAction(double distance, double velocity) {
        this(distance, velocity, 0);
    }

    public DriveStraightAction(double distance, double velocity, double heading) {
        mWantedDistance = distance;
        mVelocity = velocity;
        mHeading = heading;
    }

    @Override
    public void start() {
        startingDistance = getCurrentDistance();
        mDrive.setHighGear(false);
        mDrive.setVelocityHeadingSetpoint(mVelocity, Rotation2d.fromDegrees(mHeading));
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        boolean rv = false;
        if (mWantedDistance > 0) {
            rv = getCurrentDistance() - startingDistance >= mWantedDistance;
        } else {
            rv = getCurrentDistance() - startingDistance <= mWantedDistance;
        }
        if (rv) {
            mDrive.setVelocitySetpoint(0, 0);
        }
        return rv;
    }

    @Override
    public void done() {
        // System.out.println("Drive done, Setting drive to neutral");
        // mDrive.setOpenLoop(DriveSignal.NEUTRAL);
        mDrive.setVelocitySetpoint(0, 0);
    }

    private double getCurrentDistance() {
        return (mDrive.getLeftDistanceInches() + mDrive.getRightDistanceInches()) / 2;
    }
}
