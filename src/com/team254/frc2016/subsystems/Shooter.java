package com.team254.frc2016.subsystems;

import java.util.Collections;
import java.util.List;

import com.team254.frc2016.Constants;
import com.team254.frc2016.RobotState;
import com.team254.frc2016.loops.Loop;
import com.team254.lib.util.Rotation2d;

public class Shooter extends Subsystem {
    public static class AimingParameters {
        double range;
        Rotation2d turret_angle;

        public static class Comparator implements java.util.Comparator<AimingParameters> {
            Rotation2d mCurrentAngleInverse;

            public Comparator(Rotation2d current_turret_angle) {
                mCurrentAngleInverse = current_turret_angle.inverse();
            }

            @Override
            public int compare(AimingParameters o1, AimingParameters o2) {
                // Sort by range and distance to current turret angle
                if (o1.range < o2.range - Constants.kAutoAimRangeHysteresis) {
                    return -1;
                } else if (o1.range > o2.range + Constants.kAutoAimRangeHysteresis) {
                    return 1;
                } else {
                    // Compare to current turret angle
                    double relative_angle_o1 = Math
                            .abs(mCurrentAngleInverse.rotateBy(o1.getTurretAngle()).getRadians());
                    double relative_angle_o2 = Math
                            .abs(mCurrentAngleInverse.rotateBy(o2.getTurretAngle()).getRadians());
                    if (relative_angle_o1 < relative_angle_o2) {
                        return -1;
                    } else if (relative_angle_o1 > relative_angle_o2) {
                        return 1;
                    } else {
                        return 0;
                    }
                }
            }
        }

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
    RobotState mRobotState = RobotState.getInstance();
    boolean mOnTarget = false;
    boolean mSeesGoal = false;

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
                // Calculate new setpoint if necessary
                if (mState == State.AUTOAIMING) {
                    List<AimingParameters> aiming_parameters = mRobotState.getAimingParameters();
                    if (aiming_parameters.size() == 0) {
                        mSeesGoal = false;
                    } else {
                        mSeesGoal = true;
                        Collections.sort(aiming_parameters, new AimingParameters.Comparator(mTurret.getAngle()));
                        mHood.setDesiredAngle(getHoodAngleForRange(aiming_parameters.get(0).getRange()));
                        mTurret.setDesiredAngle(aiming_parameters.get(0).getTurretAngle());
                        // TODO deal with out of bounds. choose other target or
                        // set error bit?
                    }
                }

                // Run hood loop
                mHood.getLoop().onLoop();

                // Check if we are ready to transition state
                if (mState == State.WANTS_TO_STOW && mTurret.isOnTarget() && mHood.isOnTarget()) {
                    stowNow();
                } else if (mState == State.BATTER || mState == State.AUTOAIMING) {
                    if (mTurret.isOnTarget() && mHood.isOnTarget() && mFlywheel.isOnTarget()) {
                        mOnTarget = true;
                    } else {
                        mOnTarget = false;
                    }
                }
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

    Rotation2d getHoodAngleForRange(double range) {
        // TODO implement using InterpolatingTreeMap
        return Rotation2d.fromDegrees(40.0);
    }

    public synchronized void stow() {
        mState = State.WANTS_TO_STOW;
        mFlywheel.stop();
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kHoodNeutralAngle));
        mTurret.setDesiredAngle(new Rotation2d());
    }

    public synchronized void stowNow() {
        mHood.setStowed(true);
        mState = State.STOWED;
        stop();
    }

    public synchronized void batterMode() {
        mHood.setStowed(false);
        mState = State.BATTER;
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kMinHoodAngle));
        mFlywheel.setRpm(Constants.kFlywheelBatterRpmSetpoint);
        mTurret.setDesiredAngle(new Rotation2d());
    }

    public synchronized void autoAim() {
        mHood.setStowed(false);
        mState = State.AUTOAIMING;
        mFlywheel.setRpm(Constants.kFlywheelAutoAimRpmSetpoint);
    }

    public synchronized void setManualMode() {
        stop();
        mState = State.MANUAL;
    }

    public synchronized void moveHoodOpenLoop(double power) {
        if (mState == State.MANUAL) {
            mHood.setOpenLoop(power);
        }
    }

    public synchronized void deployHood() {
        if (mState == State.MANUAL) {
            mHood.setStowed(false);
        }
    }

    public synchronized void moveTurretOpenLoop(double power) {
        if (mState == State.MANUAL || (mState == State.AUTOAIMING && !mSeesGoal)) {
            mTurret.setOpenLoop(power);
        }
    }

    public synchronized void setFlywheelSpeedOpenLoop(double power) {
        if (mState == State.MANUAL) {
            mFlywheel.setOpenLoop(power);
        }
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

    public synchronized void resetTurretAtMax() {
        mTurret.reset(Rotation2d.fromDegrees(Constants.kMaxTurretAngle));
    }

    public synchronized void resetTurretAtMin() {
        mTurret.reset(Rotation2d.fromDegrees(Constants.kMinTurretAngle));
    }

    public synchronized void zeroTurret() {
        mTurret.reset(new Rotation2d());
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

    public boolean isOnTarget() {
        return mOnTarget;
    }

}
