package com.team254.frc2016.auto.actions;

import com.team254.frc2016.subsystems.Superstructure;
import com.team254.lib.util.Rotation2d;

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
        double error = Math.abs(mSuperstructure.getTurret().getAngle().inverse().rotateBy(mHintAngle).getDegrees());
        return (mIsDone && error < kTolerance);
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
