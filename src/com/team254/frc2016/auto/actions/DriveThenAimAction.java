package com.team254.frc2016.auto.actions;

import java.util.List;

import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Shooter;
import com.team254.frc2016.subsystems.ShooterAimingParameters;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Rotation2d;

public class DriveThenAimAction implements Action {

    private double mStartingDistance;
    private double mDistanceUntilDeploy;
    private double mMinRange;
    private double mMaxDistance;
    private double mVelocity;
    private Drive mDrive = Drive.getInstance();
    private Shooter mShooter = Shooter.getInstance();

    public static DriveThenAimAction makeForCommonConsts() {
        return new DriveThenAimAction(140, 100, 210, 30);
    }

    public DriveThenAimAction(double distance_until_deploy, double min_range, double max_distance, double velocity) {
        this.mDistanceUntilDeploy = distance_until_deploy;
        this.mMinRange = min_range;
        this.mMaxDistance = max_distance;
        this.mVelocity = velocity;
    }

    @Override
    public void start() {
        mStartingDistance = getCurrentDistance();
        mDrive.setHighGear(false);
        mDrive.setVelocityHeadingSetpoint(mVelocity, new Rotation2d());
        mShooter.setWantedState(Shooter.WantedState.WANT_TO_STOW);
    }

    @Override
    public void update() {
        double cur_distance = getCurrentDistance();
        if (cur_distance > mDistanceUntilDeploy) {
            mShooter.setWantedState(Shooter.WantedState.WANT_TO_AIM);
        }
    }

    @Override
    public boolean isFinished() {
        if (getCurrentDistance() - mStartingDistance >= mMaxDistance) {
            System.out.println("Stopping for distance");
            return true;
        }
        List<ShooterAimingParameters> params = mShooter.getCachedAimingParams();
        return (params.size() > 0 && params.get(0).getRange() <= mMinRange);
    }

    @Override
    public void done() {
        System.out.println("Drive done, stopping drive");
        mDrive.setOpenLoop(DriveSignal.NEUTRAL);
    }

    private double getCurrentDistance() {
        return (mDrive.getLeftDistanceInches() + mDrive.getRightDistanceInches()) / 2;
    }
}
