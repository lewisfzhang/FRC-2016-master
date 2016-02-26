package com.team254.frc2016.subsystems;

import java.util.List;

import com.team254.frc2016.Constants;
import com.team254.frc2016.GoalTracker;
import com.team254.frc2016.RobotState;
import com.team254.frc2016.loops.Loop;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
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
        REENABLED, // The shooter has just been enabled (and may have previously
                   // been disabled from any state)
        STOWED_AND_HOMING_HOOD, // The shooter is stowed and the hood is homing
        STOWED_OR_STOWING, // The shooter is stowed (or stowing)
        UNSTOWING, // The shooter is in the process of unstowing
        SPINNING_AIM, // The shooter is auto aiming
        SPINNING_BATTER, // The shooter is in batter mode
        FIRING_AIM, // The shooter is firing in auto aim mode
        FIRING_BATTER, // The shooter is firing in batter mode
        UNSTOWED_RETURNING_TO_SAFE // The shooter is preparing to stow
    }

    /**
     * Drives state changes. Outputs should not be decided based on this enum.
     */
    public enum WantedState {
        WANT_TO_STOW, // The user wants to stow the shooter
        WANT_TO_AIM, // The user wants to auto aim
        WANT_TO_BATTER // The user wants to use batter mode
    }

    /**
     * Orthogonal to the shooter state is the state of the firing mechanism.
     * Outputs should not be decided based on this enum.
     */
    public enum WantedFiringState {
        WANT_TO_HOLD_FIRE, // The user does not wish to fire
        WANT_TO_FIRE_NOW, // The user wants to fire now, regardless of readiness
        WANT_TO_FIRE_WHEN_READY // The user wants to fire as soon as we achieve
                                // readiness
    }

    private WantedState mWantedState = WantedState.WANT_TO_STOW;
    private WantedFiringState mWantedFiringState = WantedFiringState.WANT_TO_HOLD_FIRE;

    // Every time we transition states, we update the current state start time
    // and the state changed boolean (for one cycle)
    private double mCurrentStateStartTime;
    private boolean mStateChanged;

    private double mCurrentRangeForLogging;
    private double mCurrentAngleForLogging;
    private SystemState mSystemStateForLogging;

    private double mTurretManualScanOutput = 0;
    int mCurrentTrackId = -1;

    Turret mTurret = new Turret();
    Flywheel mFlywheel = new Flywheel();
    Hood mHood = new Hood();
    Solenoid mShooterSolenoid = new Solenoid(Constants.kShooterSolenoidId / 8, Constants.kShooterSolenoidId % 8);
    RobotState mRobotState = RobotState.getInstance();

    InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mHoodMap = Constants.kHoodAutoAimMapNewBalls;

    // NetworkTables
    NetworkTable mShooterTable = NetworkTable.getTable("shooter");

    Loop mLoop = new Loop() {

        private SystemState mSystemState = SystemState.REENABLED;

        @Override
        public void onStart() {
            synchronized (Shooter.this) {
                mHood.getLoop().onStart();
                mWantedState = WantedState.WANT_TO_STOW;
                mWantedFiringState = WantedFiringState.WANT_TO_HOLD_FIRE;
                mCurrentStateStartTime = Timer.getFPGATimestamp();
                mSystemState = SystemState.REENABLED;
                mStateChanged = true;
            }
        }

        @Override
        public void onLoop() {
            synchronized (Shooter.this) {
                mHood.getLoop().onLoop();
                double now = Timer.getFPGATimestamp();
                SystemState newState;
                switch (mSystemState) {
                case REENABLED:
                    newState = handleReenabled();
                    break;
                case STOWED_AND_HOMING_HOOD:
                    newState = handleStowedAndHomingHood();
                    break;
                case STOWED_OR_STOWING:
                    newState = handleStowedOrStowing();
                    break;
                case UNSTOWING:
                    newState = handleUnstowing(now);
                    break;
                case SPINNING_AIM:
                    newState = handleSpinningAim(now);
                    break;
                case SPINNING_BATTER:
                    newState = handleSpinningBatter(now);
                    break;
                case FIRING_AIM: // fallthrough
                case FIRING_BATTER:
                    newState = handleShooting(mSystemState, now);
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
                    mCurrentStateStartTime = now;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
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

    public synchronized void setNewBalls() {
        mHoodMap = Constants.kHoodAutoAimMapNewBalls;
    }

    public synchronized void setWornBalls() {
        mHoodMap = Constants.kHoodAutoAimMapWornBalls;
    }

    public synchronized void resetTurretAtMax() {
        mTurret.reset(Rotation2d.fromDegrees(Constants.kHardMaxTurretAngle));
    }

    public synchronized void resetTurretAtMin() {
        mTurret.reset(Rotation2d.fromDegrees(Constants.kHardTurretAngle));
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

    public synchronized void setWantsToFireNow() {
        mWantedFiringState = WantedFiringState.WANT_TO_FIRE_NOW;
    }

    public synchronized void setWantsToFireWhenReady() {
        mWantedFiringState = WantedFiringState.WANT_TO_FIRE_WHEN_READY;
    }

    public synchronized void setWantsToHoldFire() {
        mWantedFiringState = WantedFiringState.WANT_TO_HOLD_FIRE;
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

    private synchronized SystemState handleReenabled() {
        if (!mHood.hasHomed()) {
            // We assume that this only happens when we are first enabled
            return handleStowedAndHomingHood();
        }
        mTurret.setDesiredAngle(new Rotation2d());
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kBatterHoodAngle));
        mFlywheel.stop();
        setShooterSolenoidLift(false);

        if (mTurret.isSafe() && mHood.isSafe()) {
            return handleStowedOrStowing();
        } else {
            return handleReturningToSafe();
        }
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
        case WANT_TO_AIM: // fallthrough
        case WANT_TO_BATTER:
            return SystemState.UNSTOWING;
        case WANT_TO_STOW: // fallthrough
        default:
            // Nothing to do, these outputs are redundant
            return SystemState.STOWED_OR_STOWING;
        }
    }

    private synchronized SystemState handleUnstowing(double now) {
        mTurret.setDesiredAngle(new Rotation2d());
        mFlywheel.setOpenLoop(0);
        mHood.setStowed(false);
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kBatterHoodAngle));
        setShooterSolenoidLift(false);

        // State transitions
        switch (mWantedState) {
        case WANT_TO_AIM: // fallthrough
        case WANT_TO_BATTER:
            boolean isDoneUnstowing = now - mCurrentStateStartTime > Constants.kHoodUnstowToFlywheelSpinTime;
            if (!isDoneUnstowing) {
                return SystemState.UNSTOWING;
            } else {
                mRobotState.resetVision();
                mCurrentTrackId = -1;
                return mWantedState == WantedState.WANT_TO_AIM ? SystemState.SPINNING_AIM : SystemState.SPINNING_BATTER;
            }
        case WANT_TO_STOW: // fallthrough
        default:
            return SystemState.STOWED_OR_STOWING;
        }
    }

    private synchronized SystemState handleSpinningAim(double now) {
        if (mStateChanged) {
            // Start flywheel
            mFlywheel.setRpm(Constants.kFlywheelAutoAimNominalRpmSetpoint);
        }

        mHood.setStowed(false);
        setShooterSolenoidLift(false);
        autoAim(now, true);

        // State transition
        switch (mWantedState) {
        case WANT_TO_AIM:
            if (mWantedFiringState == WantedFiringState.WANT_TO_FIRE_NOW
                    || (mWantedFiringState == WantedFiringState.WANT_TO_FIRE_WHEN_READY
                            && readyToFire(SystemState.FIRING_AIM, now))) {
                return SystemState.FIRING_AIM;
            } else {
                return SystemState.SPINNING_AIM;
            }
        case WANT_TO_BATTER:
            return SystemState.SPINNING_BATTER;
        case WANT_TO_STOW: // fallthrough
        default:
            return SystemState.UNSTOWED_RETURNING_TO_SAFE;
        }
    }

    private synchronized SystemState handleSpinningBatter(double now) {
        mTurret.setDesiredAngle(new Rotation2d());
        mFlywheel.setRpm(Constants.kFlywheelBatterRpmSetpoint);
        mHood.setStowed(false);
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kBatterHoodAngle));
        setShooterSolenoidLift(false);

        // state transitions
        switch (mWantedState) {
        case WANT_TO_AIM:
            return SystemState.SPINNING_AIM;
        case WANT_TO_BATTER:
            if (mWantedFiringState == WantedFiringState.WANT_TO_FIRE_NOW
                    || (mWantedFiringState == WantedFiringState.WANT_TO_FIRE_WHEN_READY
                            && readyToFire(SystemState.FIRING_AIM, now))) {
                return SystemState.FIRING_BATTER;
            } else {
                return SystemState.SPINNING_BATTER;
            }
        case WANT_TO_STOW: // fallthrough
        default:
            return SystemState.UNSTOWED_RETURNING_TO_SAFE;
        }
    }

    private synchronized SystemState handleShooting(SystemState state, double now) {
        if (state == SystemState.FIRING_AIM) {
            autoAim(now, false);
        }

        if (Timer.getFPGATimestamp() - mCurrentStateStartTime < Constants.kShootActuationTime) {
            setShooterSolenoidLift(true);
            return state;
        } else {
            setShooterSolenoidLift(false);

            switch (mWantedState) {
            case WANT_TO_AIM:
                return SystemState.SPINNING_AIM;
            case WANT_TO_BATTER:
                return SystemState.FIRING_BATTER;
            case WANT_TO_STOW: // fallthrough
            default:
                return SystemState.UNSTOWED_RETURNING_TO_SAFE;
            }
        }
    }

    private synchronized SystemState handleReturningToSafe() {
        mTurret.setDesiredAngle(new Rotation2d());
        mFlywheel.stop();
        mHood.setStowed(false);
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kBatterHoodAngle));
        setShooterSolenoidLift(false);

        switch (mWantedState) {
        case WANT_TO_AIM:
            return SystemState.SPINNING_AIM;
        case WANT_TO_BATTER:
            return SystemState.SPINNING_BATTER;
        default:
            return mTurret.isSafe() && mHood.isSafe() ? SystemState.STOWED_OR_STOWING
                    : SystemState.UNSTOWED_RETURNING_TO_SAFE;
        }
    }

    private void setShooterSolenoidLift(boolean shouldLift) {
        mShooterSolenoid.set(shouldLift);
    }

    private double getHoodAngleForRange(double range) {
        return mHoodMap.getInterpolated(new InterpolatingDouble(range)).value;
    }

    private static double getRpmForRange(double range) {
        return Constants.kFlywheelAutoAimNominalRpmSetpoint;
    }

    private void autoAim(double now, boolean allow_changing_tracks) {
        List<ShooterAimingParameters> aimingParameters = mRobotState.getAimingParameters(now,
                new GoalTracker.TrackReportComparator(Constants.kTrackReportComparatorStablityWeight,
                        Constants.kTrackReportComparatorAgeWeight, Constants.kTrackReportComparatorSwitchingWeight,
                        mCurrentTrackId, now));
        if (aimingParameters.isEmpty()) {
            // Manual search
            System.out.println("No targets");
            mTurret.setOpenLoop(mTurretManualScanOutput);
            mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kHoodNeutralAngle));
        } else {
            System.out.println("Picking a target");
            // Pick the target to aim at
            for (ShooterAimingParameters param : aimingParameters) {
                double turret_angle_degrees = param.getTurretAngle().getDegrees();
                if (turret_angle_degrees >= Constants.kSoftMinTurretAngle
                        && turret_angle_degrees <= Constants.kSoftMaxTurretAngle
                        && param.getRange() >= Constants.kAutoAimMinRange
                        && param.getRange() <= Constants.kAutoAimMaxRange
                        && (allow_changing_tracks || mCurrentTrackId == param.getTrackid())) {
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
    }

    private boolean readyToFire(SystemState state, double now) {
        // TODO: Consider applying time hysteresis (require being ready for > X
        // ms consecutively)
        if (state == SystemState.SPINNING_AIM || state == SystemState.SPINNING_BATTER) {
            return mHood.isOnTarget() && mFlywheel.isOnTarget() && mTurret.isOnTarget();
        } else {
            return false;
        }
    }
}
