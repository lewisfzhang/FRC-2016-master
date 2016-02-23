package com.team254.frc2016.subsystems;

import java.util.Collections;
import java.util.List;

import com.team254.frc2016.Constants;
import com.team254.frc2016.RobotState;
import com.team254.frc2016.loops.Loop;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.Rotation2d;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    static final double kFlywheelDelay = 0.5;
    SubsystemState mDesiredSubsystemState = SubsystemState.STOWED;
    SubsystemState mActualSubsystemState = SubsystemState.STOWED;
    ShootState mShootState = ShootState.IDLE;
    double mShootStartTime = 0;
    Turret mTurret = new Turret();
    Flywheel mFlywheel = new Flywheel();
    Hood mHood = new Hood();
    Solenoid mShooterSolenoid = new Solenoid(Constants.kShooterSolenoidId / 8, Constants.kShooterSolenoidId % 8);
    RobotState mRobotState = RobotState.getInstance();
    boolean mOnTarget = false;
    boolean mSeesGoal = false;
    double mHoodDeployTime = 0;
    InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mHoodAutoAimMap = new InterpolatingTreeMap<>();
    double mHoodAutoAimBias = 0;

    double mCurrentRange = 0;
    double mCurrentAngle = 0;

    Loop mLoop = new Loop() {
        @Override
        public void onStart() {
            synchronized (Shooter.this) {
                mActualSubsystemState = SubsystemState.STOWED;
                mShootState = ShootState.IDLE;
                mHood.getLoop().onStart();
            }
        }

        @Override
        public void onLoop() {
            synchronized (Shooter.this) {
                double now = Timer.getFPGATimestamp();
                // Shooter state machine
                if (mActualSubsystemState != mDesiredSubsystemState) {
                    // Lock out state changes if we are actively shooting
                    if (mShootState != ShootState.SHOOTING) {
                        if (mDesiredSubsystemState == SubsystemState.AUTOAIMING) {
                            System.out.println("Enter state AUTOAIMING");
                            // Start AUTOAIMING
                            Intake.getInstance().overrideIntaking(true);
                            mRobotState.resetVision();
                            mFlywheel.stop();
                            mHood.setStowed(false);
                            mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kHoodNeutralAngle));
                            mHoodDeployTime = now;
                            mActualSubsystemState = mDesiredSubsystemState;
                        } else if (mDesiredSubsystemState == SubsystemState.BATTER) {
                            System.out.println("Enter state BATTER");
                            // Start BATTER
                            Intake.getInstance().overrideIntaking(true);
                            mFlywheel.setRpm(Constants.kFlywheelBatterRpmSetpoint);
                            mHood.setStowed(false);
                            mHoodDeployTime = now;
                            mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kBatterHoodAngle));
                            mTurret.setDesiredAngle(new Rotation2d());
                            mActualSubsystemState = mDesiredSubsystemState;
                        } else if (mDesiredSubsystemState == SubsystemState.MANUAL) {
                            System.out.println("Enter state MANUAL");
                            // Start MANUAL
                            Intake.getInstance().overrideIntaking(true);
                            stop();
                            mHood.setStowed(false);
                            mHoodDeployTime = now;
                            mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kMinHoodAngle));
                            mActualSubsystemState = mDesiredSubsystemState;
                        } else if (mDesiredSubsystemState == SubsystemState.WANTS_TO_STOW
                                && mActualSubsystemState != SubsystemState.STOWED) {
                            System.out.println("Enter state WANTS_TO_STOW");
                            Intake.getInstance().overrideIntaking(true);
                            // Start WANTS_TO_STOW
                            mActualSubsystemState = SubsystemState.WANTS_TO_STOW;
                            mFlywheel.stop();
                            mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kHoodNeutralAngle));
                            mTurret.setDesiredAngle(new Rotation2d());
                        }
                    }
                }
                if (mActualSubsystemState == SubsystemState.WANTS_TO_STOW && mTurret.isOnTarget()
                        && mHood.isOnTarget()) {
                    System.out.println("Enter state STOWED");
                    // WANTS_TO_STOW -> STOWED
                    Intake.getInstance().overrideIntaking(false);
                    stowNow();
                }

                // Calculate new setpoint if necessary
                if (mActualSubsystemState == SubsystemState.AUTOAIMING && mShootState != ShootState.SHOOTING) {
                    if (now - mHoodDeployTime < kFlywheelDelay) {
                        mRobotState.resetVision();
                    } else {
                        List<AimingParameters> aiming_parameters = mRobotState.getAimingParameters(now);
                        mSeesGoal = false;
                        if (aiming_parameters.size() > 0) {
                            Collections.sort(aiming_parameters, new AimingParameters.Comparator(mTurret.getAngle()));
                            for (AimingParameters param : aiming_parameters) {
                                // Deal with parameters outside of the shooter
                                // range
                                double turret_angle_degrees = param.getTurretAngle().getDegrees();
                                if (turret_angle_degrees >= Constants.kMinTurretAngle
                                        && turret_angle_degrees <= Constants.kMaxTurretAngle
                                        && param.getRange() >= Constants.kAutoAimMinRange
                                        && param.getRange() <= Constants.kAutoAimMaxRange) {
                                    mCurrentRange = param.getRange();
                                    mCurrentAngle = param.getTurretAngle().getDegrees();
                                    mFlywheel.setRpm(getRpmForRange(param.getRange()));
                                    mHood.setDesiredAngle(Rotation2d
                                            .fromDegrees(getHoodAngleForRange(param.getRange()) + mHoodAutoAimBias));
                                    mTurret.setDesiredAngle(param.getTurretAngle());
                                    mSeesGoal = true;
                                    break;
                                }
                            }
                        }
                    }
                }

                // Run hood loop
                mHood.getLoop().onLoop();

                // Check if we are on target
                if (mActualSubsystemState == SubsystemState.BATTER
                        || mActualSubsystemState == SubsystemState.AUTOAIMING) {
                    if (mTurret.isOnTarget() && mHood.isOnTarget() && mFlywheel.isOnTarget()) {
                        mOnTarget = true;
                    } else {
                        mOnTarget = false;
                    }
                }

                // Shooting action
                if ((mShootState == ShootState.START_AUTO_SHOOT && mOnTarget && mSeesGoal)
                        || mShootState == ShootState.START_SHOOT_NOW) {
                    mShootStartTime = now;
                    mShootState = ShootState.SHOOTING;
                    mShooterSolenoid.set(true);
                }
                if (mShootState == ShootState.SHOOTING) {
                    mFlywheel.setOpenLoop(1.0);
                    if (now - mShootStartTime >= Constants.kShootActuationTime) {
                        mShootState = ShootState.IDLE;
                        mShooterSolenoid.set(false);
                    }
                }
            }
        }

        @Override
        public void onStop() {
            synchronized (Shooter.this) {
                mActualSubsystemState = SubsystemState.STOWED;
                mShootState = ShootState.IDLE;
                mHood.getLoop().onStop();
                mFlywheel.stop();
                mHood.stop();
                mTurret.stop();
            }
        }
    };

    private Shooter() {
        mHoodAutoAimMap.put(new InterpolatingDouble(56.0), new InterpolatingDouble(41.5));
        mHoodAutoAimMap.put(new InterpolatingDouble(57.0), new InterpolatingDouble(42.0));
        mHoodAutoAimMap.put(new InterpolatingDouble(70.0), new InterpolatingDouble(47.0));
        mHoodAutoAimMap.put(new InterpolatingDouble(75.0), new InterpolatingDouble(49.0));
        mHoodAutoAimMap.put(new InterpolatingDouble(82.0), new InterpolatingDouble(50.0));
        mHoodAutoAimMap.put(new InterpolatingDouble(90.0), new InterpolatingDouble(54.0));
        mHoodAutoAimMap.put(new InterpolatingDouble(98.0), new InterpolatingDouble(55.0));
        mHoodAutoAimMap.put(new InterpolatingDouble(113.0), new InterpolatingDouble(56.5));
    }

    public Loop getLoop() {
        return mLoop;
    }

    double getHoodAngleForRange(double range) {
        return mHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
    }

    double getRpmForRange(double range) {
        return Constants.kFlywheelAutoAimNominalRpmSetpoint;
    }

    public synchronized void wantStow() {
        mDesiredSubsystemState = SubsystemState.WANTS_TO_STOW;
    }

    public synchronized void stowNow() {
        mActualSubsystemState = mDesiredSubsystemState = SubsystemState.STOWED;
        mHood.setStowed(true);
        stop();
    }

    public synchronized void wantBatterMode() {
        mDesiredSubsystemState = SubsystemState.BATTER;
    }

    public synchronized void wantAutoAim() {
        mDesiredSubsystemState = SubsystemState.AUTOAIMING;
    }

    public synchronized void wantManualMode() {
        mDesiredSubsystemState = SubsystemState.MANUAL;
    }

    public synchronized void moveHoodManual(double power) {
        if (mActualSubsystemState == SubsystemState.MANUAL) {
            mHood.setOpenLoop(power);
        }
    }

    public synchronized void moveTurretManual(double power) {
        if (mActualSubsystemState == SubsystemState.MANUAL
                || (mActualSubsystemState == SubsystemState.AUTOAIMING && !mSeesGoal)) {
            mTurret.setOpenLoop(power);
        }
    }

    public synchronized void setFlywheelSpeedManual(double power) {
        if (mActualSubsystemState == SubsystemState.MANUAL) {
            mFlywheel.setOpenLoop(power);
        }
    }

    public synchronized void wantShootNow() {
        if (mActualSubsystemState != SubsystemState.STOWED && mActualSubsystemState != SubsystemState.WANTS_TO_STOW) {
            if (mShootState == ShootState.IDLE || mShootState == ShootState.START_AUTO_SHOOT) {
                mShootState = ShootState.START_SHOOT_NOW;
            }
        }
    }

    public synchronized void setAutoShoot(boolean on) {
        if (mActualSubsystemState != SubsystemState.STOWED && mActualSubsystemState != SubsystemState.WANTS_TO_STOW) {
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
        SmartDashboard.putNumber("hood_bias", mHoodAutoAimBias);
        SmartDashboard.putNumber("current_range", mCurrentRange);
        SmartDashboard.putNumber("current_angle", mCurrentAngle);
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

    public synchronized void setHoodBias(double bias) {
        mHoodAutoAimBias = bias;
    }

    public synchronized double getHoodBias() {
        return mHoodAutoAimBias;
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

    public boolean isIdle() {
        return mShootState == ShootState.IDLE;
    }
}
