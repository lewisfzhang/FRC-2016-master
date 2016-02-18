package com.team254.frc2016.subsystems;

import java.util.Collections;
import java.util.List;

import com.team254.frc2016.Constants;
import com.team254.frc2016.RobotState;
import com.team254.frc2016.loops.Loop;
import com.team254.lib.util.Rotation2d;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

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
                    // It is theoretically possible that this computation will
                    // assume rotation opposite the hard stops on the turret,
                    // but this is basically guaranteed not to happen because of
                    // the FOV of the camera relative to turret rotation.
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

    public enum SubsystemState {
        WANTS_TO_STOW, STOWED, AUTOAIMING, BATTER, MANUAL
    }

    public enum ShootState {
        IDLE, START_SHOOT_NOW, START_AUTO_SHOOT, SHOOTING
    }

    SubsystemState mSubsystemState = SubsystemState.STOWED;
    ShootState mShootState = ShootState.IDLE;
    double mShootStartTime = 0;
    Turret mTurret = new Turret();
    Flywheel mFlywheel = new Flywheel();
    Hood mHood = new Hood();
    Solenoid mShooterSolenoid = new Solenoid(Constants.kShooterSolenoidId);
    RobotState mRobotState = RobotState.getInstance();
    boolean mOnTarget = false;
    boolean mSeesGoal = false;

    Loop mLoop = new Loop() {
        @Override
        public void onStart() {
            synchronized (Shooter.this) {
                mSubsystemState = SubsystemState.STOWED;
                mShootState = ShootState.IDLE;
                mHood.getLoop().onStart();
            }
        }

        @Override
        public void onLoop() {
            synchronized (Shooter.this) {
                // Calculate new setpoint if necessary
                if (mSubsystemState == SubsystemState.AUTOAIMING) {
                    List<AimingParameters> aiming_parameters = mRobotState.getAimingParameters();
                    mSeesGoal = false;
                    if (aiming_parameters.size() > 0) {
                        Collections.sort(aiming_parameters, new AimingParameters.Comparator(mTurret.getAngle()));
                        for (AimingParameters param : aiming_parameters) {
                            // Deal with parameters outside of the shooter range
                            double turret_angle_degrees = param.getTurretAngle().getDegrees();
                            if (turret_angle_degrees >= Constants.kMinTurretAngle
                                    && turret_angle_degrees <= Constants.kMaxTurretAngle
                                    && param.getRange() >= Constants.kAutoAimMinRange
                                    && param.getRange() <= Constants.kAutoAimMaxRange) {
                                mFlywheel.setRpm(getRpmForRange(param.getRange()));
                                mHood.setDesiredAngle(getHoodAngleForRange(param.getRange()));
                                mTurret.setDesiredAngle(param.getTurretAngle());
                                mSeesGoal = true;
                                break;
                            }
                        }
                    }
                }

                // Run hood loop
                mHood.getLoop().onLoop();

                // Check if we are ready to transition state
                if (mSubsystemState == SubsystemState.WANTS_TO_STOW && mTurret.isOnTarget() && mHood.isOnTarget()) {
                    stowNow();
                } else if (mSubsystemState == SubsystemState.BATTER || mSubsystemState == SubsystemState.AUTOAIMING) {
                    if (mTurret.isOnTarget() && mHood.isOnTarget() && mFlywheel.isOnTarget()) {
                        mOnTarget = true;
                    } else {
                        mOnTarget = false;
                    }
                }

                // Check shooter state
                if ((mShootState == ShootState.START_AUTO_SHOOT && mOnTarget && mSeesGoal)
                        || mShootState == ShootState.START_SHOOT_NOW) {
                    mShootStartTime = Timer.getFPGATimestamp();
                    mShootState = ShootState.SHOOTING;
                    mShooterSolenoid.set(true);
                }
                if (mShootState == ShootState.SHOOTING) {
                    if (Timer.getFPGATimestamp() - mShootStartTime >= Constants.kShootActuationTime) {
                        mShootState = ShootState.IDLE;
                        mShooterSolenoid.set(false);
                    }
                }
            }
        }

        @Override
        public void onStop() {
            synchronized (Shooter.this) {
                mSubsystemState = SubsystemState.STOWED;
                mShootState = ShootState.IDLE;
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

    double getRpmForRange(double range) {
        return Constants.kFlywheelAutoAimNominalRpmSetpoint;
    }

    public synchronized void stow() {
        mSubsystemState = SubsystemState.WANTS_TO_STOW;
        mFlywheel.stop();
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kHoodNeutralAngle));
        mTurret.setDesiredAngle(new Rotation2d());
    }

    public synchronized void stowNow() {
        mHood.setStowed(true);
        mSubsystemState = SubsystemState.STOWED;
        stop();
    }

    public synchronized void batterMode() {
        mHood.setStowed(false);
        mSubsystemState = SubsystemState.BATTER;
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kMinHoodAngle));
        mFlywheel.setRpm(Constants.kFlywheelBatterRpmSetpoint);
        mTurret.setDesiredAngle(new Rotation2d());
    }

    public synchronized void autoAim() {
        mHood.setStowed(false);
        mSubsystemState = SubsystemState.AUTOAIMING;
        mFlywheel.setRpm(Constants.kFlywheelAutoAimNominalRpmSetpoint);
    }

    public synchronized void setManualMode() {
        stop();
        mSubsystemState = SubsystemState.MANUAL;
    }

    public synchronized void moveHoodOpenLoop(double power) {
        if (mSubsystemState == SubsystemState.MANUAL) {
            mHood.setOpenLoop(power);
        }
    }

    public synchronized void deployHood() {
        if (mSubsystemState == SubsystemState.MANUAL) {
            mHood.setStowed(false);
        }
    }

    public synchronized void moveTurretOpenLoop(double power) {
        if (mSubsystemState == SubsystemState.MANUAL || (mSubsystemState == SubsystemState.AUTOAIMING && !mSeesGoal)) {
            mTurret.setOpenLoop(power);
        }
    }

    public synchronized void setFlywheelSpeedOpenLoop(double power) {
        if (mSubsystemState == SubsystemState.MANUAL) {
            mFlywheel.setOpenLoop(power);
        }
    }

    public synchronized void shootNow() {
        if (mSubsystemState != SubsystemState.STOWED || mSubsystemState != SubsystemState.WANTS_TO_STOW) {
            if (mShootState == ShootState.IDLE || mShootState == ShootState.START_AUTO_SHOOT) {
                mShootState = ShootState.START_SHOOT_NOW;
            }
        }
    }

    public synchronized void setAutoShoot(boolean on) {
        if (mSubsystemState != SubsystemState.STOWED || mSubsystemState != SubsystemState.WANTS_TO_STOW) {
            if (mShootState == ShootState.IDLE && on) {
                mShootState = ShootState.START_AUTO_SHOOT;
            } else if (mShootState == ShootState.START_AUTO_SHOOT && !on) {
                mShootState = ShootState.IDLE;
            }
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
