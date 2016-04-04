package com.team254.frc2016.auto.actions;

import com.team254.frc2016.subsystems.Superstructure;
import com.team254.lib.util.Rotation2d;

import edu.wpi.first.wpilibj.Timer;

public class PointTurretAction implements Action {
    private static final double kTolerance = 2.0;
    private Rotation2d mHintAngle;
    private boolean mIsDone;
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    public PointTurretAction(Rotation2d hint_angle) {
        mHintAngle = hint_angle;
        mIsDone = false;
    }

    @Override
    public boolean isFinished() {
        double error = Math
                .abs(mSuperstructure.getTurret().getAngle().inverse().rotateBy(mHintAngle).getDegrees());
        System.out.println("ERROR " + error);
        System.out.println("DONE? " + (mIsDone && error < kTolerance));
        return mIsDone && error < kTolerance;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        mSuperstructure.clearTurretManualPositionSetpoint();
    }

    @Override
    public void start() {
        mSuperstructure.setTurretManualPositionSetpoint(mHintAngle);
        mIsDone = true;
    }
}
