package com.team254.frc2016.subsystems;

import java.util.List;

import com.team254.frc2016.Constants;
import com.team254.frc2016.GoalTracker;
import com.team254.frc2016.RobotState;
import com.team254.frc2016.loops.Loop;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.Rotation2d;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {

    static Shooter mInstance = new Shooter();

    public static Shooter getInstance() {
        return mInstance;
    }

    /**
     * Drives actual state, all outputs should be dictated by this state
     */
    private enum SystemState {
        STOWED_AND_HOMING_HOOD, STOWED_OR_STOWING, UNSTOWING_TO_AIM, UNSTOWING_TO_BATTER, SPINNING_AIM, SPINNING_BATTER, UNSTOWED_RETURNING_TO_SAFE,
    }

    /**
     * Drives state changes. Outputs should not be decided based on this enum
     */
    public enum WantedState {
        WANT_TO_STOW, WANT_TO_AIM, WANT_TO_BATTER,
    }

    private WantedState mWantedState = WantedState.WANT_TO_STOW;

    private boolean mLastLoopWasUnstowing = false;
    private double mUnstowingStartTime;

    private double mCurrentRangeForLogging;
    private double mCurrentAngleForLogging;
    private SystemState mSystemStateForLogging;

    private double mTurretManualScanOutput = 0;
    private boolean mWantsToShoot;
    int mCurrentTrackId = -1;

    Turret mTurret = new Turret();
    Flywheel mFlywheel = new Flywheel();
    Hood mHood = new Hood();
    Solenoid mShooterSolenoid = new Solenoid(Constants.kShooterSolenoidId / 8, Constants.kShooterSolenoidId % 8);
    RobotState mRobotState = RobotState.getInstance();

    // NetworkTables
    NetworkTable mShooterTable = NetworkTable.getTable("shooter");

    Loop mLoop = new Loop() {

        private SystemState mSystemState = SystemState.STOWED_AND_HOMING_HOOD;

        @Override
        public void onStart() {
            synchronized (Shooter.this) {
                mHood.getLoop().onStart();
                mWantedState = WantedState.WANT_TO_STOW;
            }
        }

        @Override
        public void onLoop() {
            synchronized (Shooter.this) {
                mHood.getLoop().onLoop();
                double now = Timer.getFPGATimestamp();
                SystemState newState;
                switch (mSystemState) {
                case STOWED_AND_HOMING_HOOD:
                    newState = handleStowedAndHomingHood();
                    break;
                case STOWED_OR_STOWING:
                    newState = handleStowedOrStowing();
                    break;
                case UNSTOWING_TO_AIM: // fall through
                case UNSTOWING_TO_BATTER:
                    newState = handleUnstowing(now);
                    break;
                case SPINNING_AIM:
                    newState = handleSpinningAim(now);
                    break;
                case SPINNING_BATTER:
                    newState = handleSpinningBatter();
                    break;
                case UNSTOWED_RETURNING_TO_SAFE:
                    newState = handleReturningToSafe();
                    break;
                default:
                    System.out.println("Unexpected shooter state: " + mSystemState);
                    newState = SystemState.UNSTOWED_RETURNING_TO_SAFE;
                }

                if (newState != mSystemState) {
                    System.out.println("Shooter state " + mSystemState + " to " + newState);
                    mSystemState = newState;
                }
                mSystemStateForLogging = mSystemState;

                // Update Network Tables
                /*
                 * if (mOnTarget && mSeesGoal) {
                 * mShooterTable.putString("state", "LOCKED"); } else if
                 * (mSeesGoal) { mShooterTable.putString("state", "AIMING"); }
                 * else if (mActualSubsystemState == SubsystemState.STOWED) {
                 * mShooterTable.putString("state", "CAM_STOWED"); } else if
                 * (!VisionServer.getInstance().isConnected()) {
                 * mShooterTable.putString("state", "NO_CAM"); } else {
                 * mShooterTable.putString("state", "LOOKING_FOR_GOAL"); }
                 */
            }
        }

        @Override
        public void onStop() {
            synchronized (Shooter.this) {
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

    public synchronized void setWantedState(WantedState newState) {
        mWantedState = newState;
    }

    @Override
    public void outputToSmartDashboard() {
        mFlywheel.outputToSmartDashboard();
        mHood.outputToSmartDashboard();
        mTurret.outputToSmartDashboard();
        SmartDashboard.putNumber("current_range", mCurrentRangeForLogging);
        SmartDashboard.putNumber("current_angle", mCurrentAngleForLogging);
        SmartDashboard.putString("shooter_state", "" + mSystemStateForLogging);
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

    /**
     * Sets the manual turret output ONLY when the turret is auto-aiming without
     * a visible target
     */
    public synchronized void setTurretManualScanOutput(double output) {
        mTurretManualScanOutput = output;
    }

    /**
     * Controls the shooter lifter when in batter or aim mode
     */
    public synchronized void setWantsToShoot(boolean wantsToShoot) {
        mWantsToShoot = wantsToShoot;
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

    private synchronized SystemState handleStowedAndHomingHood() {
        mTurret.setDesiredAngle(new Rotation2d());
        mFlywheel.stop();
        mHood.setStowed(true);
        setShooterSolenoidLift(false);

        return mHood.hasHomed() ? SystemState.STOWED_OR_STOWING : SystemState.STOWED_AND_HOMING_HOOD;
    }

    private synchronized SystemState handleStowedOrStowing() {
        mTurret.setDesiredAngle(new Rotation2d());
        mFlywheel.stop();
        mHood.setStowed(true);
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kBatterHoodAngle));
        setShooterSolenoidLift(false);

        switch (mWantedState) {
        case WANT_TO_STOW:
            // Nothing to do, these outputs are redundant
            return SystemState.STOWED_OR_STOWING;
        case WANT_TO_AIM:
            return SystemState.UNSTOWING_TO_AIM;
        case WANT_TO_BATTER:
            return SystemState.UNSTOWING_TO_BATTER;
        }
        throw new ImpossibleException();
    }

    private synchronized SystemState handleUnstowing(double now) {
        if (!mLastLoopWasUnstowing) {
            mUnstowingStartTime = now;
            mLastLoopWasUnstowing = true;
        }

        mTurret.setDesiredAngle(new Rotation2d());
        mFlywheel.setOpenLoop(0);
        mHood.setStowed(false);
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kBatterHoodAngle));
        setShooterSolenoidLift(false);

        // State transitions

        switch (mWantedState) {
        case WANT_TO_STOW:
            mLastLoopWasUnstowing = false;
            return SystemState.STOWED_OR_STOWING;
        case WANT_TO_AIM: // fallthrough
        case WANT_TO_BATTER:
            boolean isDoneUnstowing = now - mUnstowingStartTime > Constants.kHoodUnstowToFlywheelSpinTime;
            if (!isDoneUnstowing) {
                return mWantedState == WantedState.WANT_TO_AIM ? SystemState.UNSTOWING_TO_AIM
                        : SystemState.UNSTOWING_TO_BATTER;
            }
            mRobotState.resetVision();
            mCurrentTrackId = -1;
            mLastLoopWasUnstowing = false;
            return mWantedState == WantedState.WANT_TO_AIM ? SystemState.SPINNING_AIM : SystemState.SPINNING_BATTER;
        }
        throw new ImpossibleException();
    }

    private synchronized SystemState handleSpinningAim(double now) {
        mHood.setStowed(false);
        // TODO: need to prevent changing state during the shooting action, and
        // ensure that the shooting solenoid is energized for a controlled
        // period of time.
        setShooterSolenoidLift(mWantsToShoot);
        List<ShooterAimingParameters> aimingParameters = mRobotState.getAimingParameters(now,
                new GoalTracker.TrackReportComparator(Constants.kTrackReportComparatorStablityWeight,
                        Constants.kTrackReportComparatorAgeWeight, Constants.kTrackReportComparatorSwitchingWeight,
                        mCurrentTrackId, now));
        if (aimingParameters.isEmpty()) {
            // Manual search
            System.out.println("No targets");
            mTurret.setOpenLoop(mTurretManualScanOutput);
            // TODO: another option is to just leave the flywheel and hood at
            // their last values
            mFlywheel.setOpenLoop(0);
            mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kHoodNeutralAngle));
        } else {
            System.out.println("Picking a target");
            // Pick the target to aim at
            for (ShooterAimingParameters param : aimingParameters) {
                double turret_angle_degrees = param.getTurretAngle().getDegrees();
                if (turret_angle_degrees >= Constants.kMinTurretAngle
                        && turret_angle_degrees <= Constants.kMaxTurretAngle
                        && param.getRange() >= Constants.kAutoAimMinRange
                        && param.getRange() <= Constants.kAutoAimMaxRange) {
                    // This target works
                    mFlywheel.setRpm(getRpmForRange(param.getRange()));
                    mHood.setDesiredAngle(Rotation2d.fromDegrees(getHoodAngleForRange(param.getRange())));
                    mTurret.setDesiredAngle(param.getTurretAngle());
                    mCurrentAngleForLogging = param.getTurretAngle().getDegrees();
                    mCurrentRangeForLogging = param.getRange();
                    mCurrentTrackId = param.getTrackid();
                    break;
                }
            }
        }

        // State transition
        switch (mWantedState) {
        case WANT_TO_STOW:
            return SystemState.UNSTOWED_RETURNING_TO_SAFE;
        case WANT_TO_AIM:
            return SystemState.SPINNING_AIM;
        case WANT_TO_BATTER:
            return SystemState.SPINNING_BATTER;
        }
        throw new ImpossibleException();
    }

    private synchronized SystemState handleSpinningBatter() {
        mTurret.setDesiredAngle(new Rotation2d());
        mFlywheel.setRpm(Constants.kFlywheelBatterRpmSetpoint);
        mHood.setStowed(false);
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kBatterHoodAngle));
        setShooterSolenoidLift(mWantsToShoot);

        // state transitions
        switch (mWantedState) {
        case WANT_TO_STOW:
            return SystemState.UNSTOWED_RETURNING_TO_SAFE;
        case WANT_TO_AIM:
            // mRobotState.resetVision();
            return SystemState.SPINNING_AIM;
        case WANT_TO_BATTER:
            return SystemState.SPINNING_BATTER;
        }
        throw new ImpossibleException();
    }

    private synchronized SystemState handleReturningToSafe() {
        mTurret.setDesiredAngle(new Rotation2d());
        mFlywheel.setOpenLoop(0);
        mHood.setStowed(false);
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kBatterHoodAngle));
        setShooterSolenoidLift(false);

        switch (mWantedState) {
        case WANT_TO_STOW:
            return mTurret.isOnTarget() && mHood.isOnTarget() ? SystemState.STOWED_OR_STOWING
                    : SystemState.UNSTOWED_RETURNING_TO_SAFE;
        case WANT_TO_AIM:
            return SystemState.SPINNING_AIM;
        case WANT_TO_BATTER:
            return SystemState.SPINNING_BATTER;
        }
        throw new ImpossibleException();
    }

    private void setShooterSolenoidLift(boolean shouldLift) {
        mShooterSolenoid.set(shouldLift);
    }

    private static double getHoodAngleForRange(double range) {
        return Constants.HOOD_AUTO_AIM_MAP.getInterpolated(new InterpolatingDouble(range)).value;
    }

    private static double getRpmForRange(double range) {
        return Constants.kFlywheelAutoAimNominalRpmSetpoint;
    }

    private static class ImpossibleException extends RuntimeException {
    }
}
