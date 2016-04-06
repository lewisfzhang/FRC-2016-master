package com.team254.frc2016.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.team254.frc2016.Constants;
import com.team254.frc2016.GoalTracker;
import com.team254.frc2016.RobotState;
import com.team254.frc2016.loops.Loop;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.Rotation2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem {

    static Superstructure mInstance = new Superstructure();

    public static Superstructure getInstance() {
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
        LOADING, // The shooter is bringing in the ball
        SPINNING_AIM, // The shooter is auto aiming
        SPINNING_BATTER, // The shooter is in batter mode
        FIRING_AIM, // The shooter is firing in auto aim mode
        FIRING_BATTER, // The shooter is firing in batter mode
        IDLE, // The shooter is deployed but idle
        UNSTOWED_RETURNING_TO_SAFE // The shooter is preparing to stow
    }

    /**
     * Drives state changes. Outputs should not be decided based on this enum.
     */
    public enum WantedState {
        WANT_TO_IDLE, // The user is okay with leaving the hood up
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

    public enum WantedIntakeState {
        WANT_TO_STOP_INTAKE, // The user does not wish to intake balls
        WANT_TO_RUN_INTAKE, // The user wants to intake balls
        WANT_TO_EXHAUST // The user wants to exhaust balls
    }

    private WantedState mWantedState = WantedState.WANT_TO_STOW;
    private WantedFiringState mWantedFiringState = WantedFiringState.WANT_TO_HOLD_FIRE;
    private WantedIntakeState mWantedIntakeState = WantedIntakeState.WANT_TO_STOP_INTAKE;

    private double mCurrentRangeForLogging;
    private double mCurrentAngleForLogging;
    private SystemState mSystemStateForLogging;
    private boolean mTuningMode = false;

    private double mTurretManualScanOutput = 0;
    private Optional<Rotation2d> mTurretManualSetpoint = Optional.empty();
    private double mHoodManualScanOutput = 0;
    int mCurrentTrackId = -1;
    int mConsecutiveCyclesOnTarget = 0;
    int mNumShotsFired = 0;

    private List<ShooterAimingParameters> mCachedAimingParams = new ArrayList<>();

    Turret mTurret = new Turret();
    Flywheel mFlywheel = new Flywheel();
    Hood mHood = new Hood();
    HoodRoller mHoodRoller = new HoodRoller();
    Intake mIntake = new Intake();
    RobotState mRobotState = RobotState.getInstance();

    // NetworkTables
    NetworkTable mShooterTable = NetworkTable.getTable("shooter");

    Loop mLoop = new Loop() {

        private SystemState mSystemState = SystemState.REENABLED;

        // Every time we transition states, we update the current state start
        // time and the state
        // changed boolean (for one cycle)
        private double mCurrentStateStartTime;
        private boolean mStateChanged;

        @Override
        public void onStart() {
            synchronized (Superstructure.this) {
                mHood.getLoop().onStart();
                mHoodRoller.getLoop().onStart();
                mWantedState = WantedState.WANT_TO_STOW;
                mWantedFiringState = WantedFiringState.WANT_TO_HOLD_FIRE;
                mWantedIntakeState = WantedIntakeState.WANT_TO_STOP_INTAKE;
                mCurrentStateStartTime = Timer.getFPGATimestamp();
                mSystemState = SystemState.REENABLED;
                mStateChanged = true;
                mTurretManualSetpoint = Optional.empty();
            }
        }

        @Override
        public void onLoop() {
            synchronized (Superstructure.this) {
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
                    newState = handleUnstowing(now, mCurrentStateStartTime);
                    break;
                case LOADING:
                    newState = handleLoading(now, mCurrentStateStartTime);
                    break;
                case SPINNING_AIM:
                    newState = handleSpinningAim(now, mStateChanged);
                    break;
                case SPINNING_BATTER:
                    newState = handleSpinningBatter(now);
                    break;
                case FIRING_AIM: // fallthrough
                case FIRING_BATTER:
                    newState = handleShooting(mSystemState, now, mCurrentStateStartTime);
                    break;
                case IDLE:
                    newState = handleIdle();
                    break;
                case UNSTOWED_RETURNING_TO_SAFE:
                    newState = handleReturningToSafe(now, mCurrentStateStartTime);
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
                mHoodRoller.getLoop().onLoop();
            }
        }

        @Override
        public void onStop() {
            synchronized (Superstructure.this) {
                mHood.getLoop().onStop();
                mHoodRoller.getLoop().onStop();
                mFlywheel.stop();
                mHood.stop();
                mTurret.stop();
                mIntake.stop();
            }
        }
    };

    private Superstructure() {
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
        mHoodRoller.outputToSmartDashboard();
        mIntake.outputToSmartDashboard();
        SmartDashboard.putNumber("current_range", mCurrentRangeForLogging);
        SmartDashboard.putNumber("current_angle", mCurrentAngleForLogging);
        SmartDashboard.putString("shooter_state", "" + mSystemStateForLogging);
    }

    @Override
    public synchronized void stop() {
        mFlywheel.stop();
        mHood.stop();
        mHoodRoller.stop();
        mTurret.stop();
        mIntake.stop();
    }

    @Override
    public synchronized void zeroSensors() {
        mFlywheel.zeroSensors();
        mHood.zeroSensors();
        mHoodRoller.zeroSensors();
        mTurret.zeroSensors();
        mCachedAimingParams.clear();
        mIntake.zeroSensors();
    }

    public synchronized void resetTurretAtMax() {
        mTurret.reset(Rotation2d.fromDegrees(Constants.kHardMaxTurretAngle));
    }

    public synchronized void resetTurretAtMin() {
        mTurret.reset(Rotation2d.fromDegrees(Constants.kHardMinTurretAngle));
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

    public synchronized void setHoodManualScanOutput(double output) {
        mHoodManualScanOutput = output;
    }

    public synchronized void setTurretManualPositionSetpoint(Rotation2d angle) {
        mTurretManualSetpoint = Optional.of(angle);
    }

    public synchronized void clearTurretManualPositionSetpoint() {
        mTurretManualSetpoint = Optional.empty();
    }

    public void setTestServoSpeed(double speed) {
        mHood.setTestServoSpeed(speed);
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

    public synchronized void setWantsToExhaust() {
        mWantedIntakeState = WantedIntakeState.WANT_TO_EXHAUST;
    }

    public synchronized void setWantsToStopIntake() {
        mWantedIntakeState = WantedIntakeState.WANT_TO_STOP_INTAKE;
    }

    public synchronized void setWantsToRunIntake() {
        mWantedIntakeState = WantedIntakeState.WANT_TO_RUN_INTAKE;
    }

    public synchronized void deployIntake() {
        mIntake.setDeploy(true);
    }

    public synchronized void stowIntake() {
        mIntake.setDeploy(false);
    }

    public synchronized int getNumShotsFired() {
        return mNumShotsFired;
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

    public synchronized void setTuningMode(boolean tuning_on) {
        mTuningMode = tuning_on;
    }

    private synchronized SystemState handleReenabled() {
        if (!mHood.hasHomed()) {
            // We assume that this only happens when we are first enabled
            return handleStowedAndHomingHood();
        }
        mTurret.setDesiredAngle(new Rotation2d());
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kBatterHoodAngle));
        mHoodRoller.stop();
        mFlywheel.stop();

        if (mTurret.isSafe() && mHood.isSafe()) {
            return handleStowedOrStowing();
        } else {
            double now = Timer.getFPGATimestamp();
            return handleReturningToSafe(now, now);
        }
    }

    private synchronized SystemState handleStowedAndHomingHood() {
        handleIntake(false, false);
        mTurret.setDesiredAngle(new Rotation2d());
        mFlywheel.stop();
        mHood.setStowed(true);
        mHoodRoller.stop();

        return mHood.hasHomed() ? SystemState.STOWED_OR_STOWING : SystemState.STOWED_AND_HOMING_HOOD;
    }

    private synchronized SystemState handleStowedOrStowing() {
        handleIntake(false, false);
        mTurret.setDesiredAngle(new Rotation2d());
        mFlywheel.stop();
        mHood.setStowed(true);
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kBatterHoodAngle));
        if (mWantedIntakeState == WantedIntakeState.WANT_TO_EXHAUST) {
            mHoodRoller.reverse();
        } else {
            mHoodRoller.stop();
        }

        switch (mWantedState) {
        case WANT_TO_AIM: // fallthrough
        case WANT_TO_BATTER:
        case WANT_TO_IDLE:
            return SystemState.UNSTOWING;
        case WANT_TO_STOW: // fallthrough
        default:
            // Nothing to do, these outputs are redundant
            return SystemState.STOWED_OR_STOWING;
        }
    }

    private synchronized SystemState handleUnstowing(double now, double stateStartTime) {
        handleIntake(true, false);
        mTurret.setDesiredAngle(new Rotation2d());
        mFlywheel.stop();
        mHood.setStowed(false);
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kBatterHoodAngle));
        mHoodRoller.stop();

        // State transitions
        switch (mWantedState) {
        case WANT_TO_AIM: // fallthrough
        case WANT_TO_BATTER: // fallthrough
        case WANT_TO_IDLE:
            boolean isDoneUnstowing = now - stateStartTime > Constants.kHoodUnstowToFlywheelSpinTime;
            if (!isDoneUnstowing) {
                return SystemState.UNSTOWING;
            } else {
                return SystemState.LOADING;
            }
        case WANT_TO_STOW: // fallthrough
        default:
            mHood.startHoming();
            return SystemState.STOWED_OR_STOWING;
        }
    }

    private synchronized SystemState handleLoading(double now, double stateStartTime) {
        handleIntake(true, true);
        mTurret.setDesiredAngle(new Rotation2d());
        mFlywheel.setRpm(Constants.kFlywheelGoodBallRpmSetpoint);
        mHood.setStowed(false);
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kBatterHoodAngle));
        mHoodRoller.intake();

        switch (mWantedState) {
        case WANT_TO_AIM: // fallthrough
        case WANT_TO_BATTER:
        case WANT_TO_IDLE:
            boolean isDoneLoading = now - stateStartTime > Constants.kLoadingTime;
            if (isDoneLoading) {
                mRobotState.resetVision();
                mCurrentTrackId = -1;
                if (mWantedState == WantedState.WANT_TO_AIM) {
                    return SystemState.SPINNING_AIM;
                } else if (mWantedState == WantedState.WANT_TO_BATTER) {
                    return SystemState.SPINNING_BATTER;
                } else {
                    return SystemState.IDLE;
                }
            } else {
                return SystemState.LOADING;
            }
        case WANT_TO_STOW:
        default:
            return SystemState.UNSTOWED_RETURNING_TO_SAFE;
        }
    }

    private synchronized SystemState handleSpinningAim(double now, boolean isFirstCycle) {
        handleIntake(true, false);
        if (isFirstCycle) {
            // Start flywheel
            mFlywheel.setRpm(Constants.kFlywheelGoodBallRpmSetpoint);
            mConsecutiveCyclesOnTarget = 0;
        }

        mHood.setStowed(false);
        mHoodRoller.intake();
        autoAim(now, true);

        // State transition
        switch (mWantedState) {
        case WANT_TO_AIM:
            if (mWantedFiringState == WantedFiringState.WANT_TO_FIRE_NOW
                    || (mWantedFiringState == WantedFiringState.WANT_TO_FIRE_WHEN_READY
                            && readyToFire(SystemState.SPINNING_AIM, now))) {
                return SystemState.FIRING_AIM;
            } else {
                return SystemState.SPINNING_AIM;
            }
        case WANT_TO_BATTER:
            return SystemState.SPINNING_BATTER;
        case WANT_TO_IDLE:
            return SystemState.IDLE;
        case WANT_TO_STOW: // fallthrough
        default:
            return SystemState.UNSTOWED_RETURNING_TO_SAFE;
        }
    }

    private synchronized SystemState handleSpinningBatter(double now) {
        handleIntake(true, false);
        mTurret.setDesiredAngle(new Rotation2d());
        mFlywheel.setRpm(Constants.kFlywheelGoodBallRpmSetpoint);
        mHood.setStowed(false);
        mHoodRoller.intake();
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kBatterHoodAngle));

        // state transitions
        switch (mWantedState) {
        case WANT_TO_AIM:
            return SystemState.SPINNING_AIM;
        case WANT_TO_BATTER:
            if (mWantedFiringState == WantedFiringState.WANT_TO_FIRE_NOW
                    || (mWantedFiringState == WantedFiringState.WANT_TO_FIRE_WHEN_READY
                            && readyToFire(SystemState.SPINNING_BATTER, now))) {
                return SystemState.FIRING_BATTER;
            } else {
                return SystemState.SPINNING_BATTER;
            }
        case WANT_TO_IDLE:
            return SystemState.IDLE;
        case WANT_TO_STOW: // fallthrough
        default:
            return SystemState.UNSTOWED_RETURNING_TO_SAFE;
        }
    }

    private synchronized SystemState handleShooting(SystemState state, double now, double stateStartTime) {
        handleIntake(true, true);
        if (state == SystemState.FIRING_AIM) {
            autoAim(now, false);
        }

        if (now - stateStartTime < Constants.kShootActuationTime) {
            mHoodRoller.shoot();
            return state;
        } else {
            mHoodRoller.stop();
            mNumShotsFired++;
            switch (mWantedState) {
            case WANT_TO_AIM:
                return SystemState.SPINNING_AIM;
            case WANT_TO_BATTER:
                return SystemState.SPINNING_BATTER;
            case WANT_TO_IDLE:
                return SystemState.IDLE;
            case WANT_TO_STOW: // fallthrough
            default:
                return SystemState.UNSTOWED_RETURNING_TO_SAFE;
            }
        }
    }

    private synchronized SystemState handleIdle() {
        mTurret.setDesiredAngle(new Rotation2d());

        handleIntake(!mTurret.isSafe(), false);
        mFlywheel.stop();
        mHood.setStowed(false);
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kBatterHoodAngle));
        if (mWantedIntakeState == WantedIntakeState.WANT_TO_EXHAUST) {
            mHoodRoller.reverse();
        } else {
            mHoodRoller.stop();
        }

        switch (mWantedState) {
        case WANT_TO_AIM: // fallthrough
        case WANT_TO_BATTER:
            return SystemState.LOADING;
        case WANT_TO_IDLE:
            return SystemState.IDLE;
        case WANT_TO_STOW: // fallthrough
        default:
            return SystemState.UNSTOWED_RETURNING_TO_SAFE;
        }
    }

    private synchronized SystemState handleReturningToSafe(double now, double start_time) {
        handleIntake(true, false);
        mTurret.setDesiredAngle(new Rotation2d());
        mFlywheel.stop();
        mHood.setStowed(false);
        mHoodRoller.stop();
        mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kBatterHoodAngle));

        switch (mWantedState) {
        case WANT_TO_AIM:
            return SystemState.SPINNING_AIM;
        case WANT_TO_BATTER:
            return SystemState.SPINNING_BATTER;
        case WANT_TO_IDLE:
            return SystemState.IDLE;
        default:
            if ((mTurret.isSafe() && mHood.isSafe()) || (now - start_time > Constants.kStowingOverrideTime)) {
                mHood.startHoming();
                return SystemState.STOWED_OR_STOWING;
            } else {
                return SystemState.UNSTOWED_RETURNING_TO_SAFE;
            }
        }
    }

    private double getHoodAngleForRange(double range) {
        InterpolatingDouble result = Constants.kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range));
        if (result != null) {
            return result.value;
        } else {
            return Constants.kHoodNeutralAngle;
        }
    }

    private List<ShooterAimingParameters> getCurrentAimingParameters(double now) {
        List<ShooterAimingParameters> params = mRobotState.getAimingParameters(now,
                new GoalTracker.TrackReportComparator(Constants.kTrackReportComparatorStablityWeight,
                        Constants.kTrackReportComparatorAgeWeight, Constants.kTrackReportComparatorSwitchingWeight,
                        mCurrentTrackId, now));
        mCachedAimingParams = params;
        return params;

    }

    public List<ShooterAimingParameters> getCachedAimingParams() {
        return mCachedAimingParams;
    }

    private void autoAim(double now, boolean allow_changing_tracks) {
        List<ShooterAimingParameters> aimingParameters = getCurrentAimingParameters(now);
        if (aimingParameters.isEmpty() && allow_changing_tracks) {
            // Manual search
            if (mTurretManualSetpoint.isPresent()) {
                System.out.println("Going to manual setpoint");
                mTurret.setDesiredAngle(mTurretManualSetpoint.get());
            } else {
                System.out.println("No targets - Manual scan");
                mTurret.setOpenLoop(mTurretManualScanOutput);
            }
            if (!mTuningMode) {
                mHood.setDesiredAngle(Rotation2d.fromDegrees(Constants.kHoodNeutralAngle));
            } else {
                mHood.setOpenLoop(mHoodManualScanOutput);
            }
        } else {
            mHoodManualScanOutput = 0.0;
            // System.out.println("Picking a target");
            // Pick the target to aim at
            boolean has_target = false;
            for (ShooterAimingParameters param : aimingParameters) {
                double turret_angle_degrees = param.getTurretAngle().getDegrees();
                if (turret_angle_degrees >= Constants.kSoftMinTurretAngle
                        && turret_angle_degrees <= Constants.kSoftMaxTurretAngle
                        && param.getRange() >= Constants.kAutoAimMinRange
                        && param.getRange() <= Constants.kAutoAimMaxRange
                        && (allow_changing_tracks || mCurrentTrackId == param.getTrackid())) {
                    // This target works
                    mFlywheel.setRpm(getShootingSetpointRpm(param.getRange()));
                    if (!mTuningMode) {
                        mHood.setDesiredAngle(Rotation2d.fromDegrees(getHoodAngleForRange(param.getRange())));
                    } else {
                        mHood.setOpenLoop(mHoodManualScanOutput);
                    }
                    mTurret.setDesiredAngle(param.getTurretAngle());
                    // mTurret.setOpenLoop(mTurretManualScanOutput / 10.0);
                    mCurrentAngleForLogging = param.getTurretAngle().getDegrees();
                    mCurrentRangeForLogging = param.getRange();
                    mCurrentTrackId = param.getTrackid();
                    has_target = true;
                    break;
                }
            }
            if (!has_target) {
                mCurrentTrackId = -1;
            }
        }
    }

    private double getShootingSetpointRpm(double range) {
        return Constants.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
    }

    private boolean readyToFire(SystemState state, double now) {
        Drive drive = Drive.getInstance(); // TODO: hold onto a permanent
                                           // reference to this
        boolean is_stopped = Math.abs(drive.getLeftVelocityInchesPerSec()) < Constants.kAutoShootMaxDriveSpeed
                && Math.abs(drive.getRightVelocityInchesPerSec()) < Constants.kAutoShootMaxDriveSpeed;
        if (state == SystemState.SPINNING_AIM) {
            if ((mTuningMode || mHood.isOnTarget()) && mFlywheel.isOnTarget() && mTurret.isOnTarget()
                    && (mCurrentTrackId != -1)) {
                mConsecutiveCyclesOnTarget++;
            } else {
                mConsecutiveCyclesOnTarget = 0;
            }
        } else if (state == SystemState.SPINNING_BATTER) {
            if ((mTuningMode || mHood.isOnTarget()) && mFlywheel.isOnTarget() && mTurret.isOnTarget()) {
                mConsecutiveCyclesOnTarget++;
            } else {
                mConsecutiveCyclesOnTarget = 0;
            }
        } else {
            mConsecutiveCyclesOnTarget = 0;
        }
        return mConsecutiveCyclesOnTarget > Constants.kAutoAimMinConsecutiveCyclesOnTarget && is_stopped;
    }

    private void handleIntake(boolean disallow_intaking, boolean loading) {
        switch (mWantedIntakeState) {
        case WANT_TO_RUN_INTAKE:
            if (disallow_intaking) {
                mIntake.setIntakeRoller(0.0, loading ? 1.0 : 0.0);
            } else {
                mIntake.setIntakeRoller(1.0, 1.0);
            }
            break;
        case WANT_TO_EXHAUST:
            mIntake.setIntakeRoller(-1.0, loading ? 1.0 : -1.0);
            break;
        case WANT_TO_STOP_INTAKE:
        default:
            mIntake.setIntakeRoller(0.0, loading ? 1.0 : 0.0);
            break;
        }
    }
}
