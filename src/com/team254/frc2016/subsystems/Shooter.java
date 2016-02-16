package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;
import com.team254.frc2016.loops.Loop;
import com.team254.lib.util.Rotation2d;

public class Shooter extends Subsystem {
    public static class AimingParameters {
        double range;
        Rotation2d turret_angle;

        public AimingParameters(double range, Rotation2d turret_angle) {
            this.range = range;
            this.turret_angle = turret_angle;
        }

        public double getRange() {
            return range;
        }

        public Rotation2d getTurretAngle() {
            return turret_angle;
        }
    }

    static Shooter mInstance = new Shooter();

    public static Shooter getInstance() {
        return mInstance;
    }

    public enum State {
        WANTS_TO_STOW, STOWED, AUTOAIMING, BATTER, MANUAL
    }

    State mState = State.STOWED;
    Turret mTurret = new Turret();
    Flywheel mFlywheel = new Flywheel();
    Hood mHood = new Hood();

    Loop mLoop = new Loop() {
        @Override
        public void onStart() {
            synchronized (Shooter.this) {
                mState = State.STOWED;
                mHood.getLoop().onStart();
            }
        }

        @Override
        public void onLoop() {
            synchronized (Shooter.this) {
                if (mState == State.WANTS_TO_STOW && mTurret.isOnTarget() && mHood.isOnTarget()) {
                    // TODO stow the hood
                } else if (mState == State.BATTER) {
                    // TODO update on target
                } else if (mState == State.AUTOAIMING) {
                    // TODO update aim & on target
                }
                mHood.getLoop().onLoop();
            }
        }

        @Override
        public void onStop() {
            synchronized (Shooter.this) {
                mState = State.STOWED;
                mHood.getLoop().onStop();
                mFlywheel.stop();
                mHood.stop();
                mTurret.stop();
            }
        }
    };

    private Shooter() {
    }

    public Loop getLoop() {
        return mLoop;
    }

    public synchronized void stow() {
        mState = State.WANTS_TO_STOW;
        mFlywheel.stop();
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kHoodNeutralAngle));
        mTurret.setDesiredAngle(new Rotation2d());
    }

    public synchronized void stowNow() {
        mState = State.STOWED;
        mFlywheel.stop();
        mHood.stop();
        mTurret.stop();
        // TODO stow the hood
    }

    public synchronized void batterMode() {
        mState = State.BATTER;
        // TODO batter mode
    }

    public synchronized void autoAim() {
        mState = State.AUTOAIMING;
        // TODO auto aim mode
    }

    @Override
    public void outputToSmartDashboard() {
        mFlywheel.outputToSmartDashboard();
        mHood.outputToSmartDashboard();
        mTurret.outputToSmartDashboard();
    }

    @Override
    public synchronized void stop() {
        mFlywheel.stop();
        mHood.stop();
        mTurret.stop();
    }

    @Override
    public synchronized void zeroSensors() {
        mFlywheel.zeroSensors();
        mHood.zeroSensors();
        mTurret.zeroSensors();
    }

    public Turret getTurret() {
        return mTurret;
    }

    public Flywheel getFlywheel() {
        return mFlywheel;
    }

    public Hood getHood() {
        return mHood;
    }

}
