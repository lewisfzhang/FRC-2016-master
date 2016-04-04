package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;
import com.team254.frc2016.loops.Loop;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

/**
 * Controls the CDF, Portcullis, and Hanging mechanisms
 */
public class UtilityArm extends Subsystem {

    private static UtilityArm sInstance = new UtilityArm();

    public static UtilityArm getInstance() {
        return sInstance;
    }

    public enum WantedState {
        /** Keep in the sizing box, only valid from the start of the match */
        STAY_IN_SIZE_BOX,

        PORTCULLIS, DRIVING, CDF,

        /**
         * Want to get the hooks up for hanging, but not actually hang quite yet
         */
        PREPARE_FOR_HANG,

        /** Want to pull up to hang now */
        PULL_UP_HANG,

        /** Want to get arm out of the way to challenge on the batter */
        BATTER_CHALLENGE,

    }

    private enum SystemState {
        /** Start of match state */
        SIZE_BOX,

        /** Dropping the arm past the adj hardstops */
        SIZE_BOX_TO_PORTCULLIS,

        /** Lower intake to prevent porcullis mode from damaging it */
        DRIVE_TO_PORTCULLIS,

        /** Dragging on the floor, past the adj hardstops */
        PORTCULLIS,

        /** Pushing arm up against adj hardstops */
        DRIVE,

        /** Pushing arm up against adj hardstops, CDF flap deployed */
        CDF,

        /** Retracting CDF flap (to clear bumper) */
        CDF_TO_DRIVING,

        /** Lift arm past adj hardstops, clearing bumper */
        LIFTING_ARM_FOR_HANG,

        /**
         * Arm past adj hardstops and bumper, deploy CDF flap for hook clearance
         */
        OPENING_CDF_FOR_HANG,

        /** Extend hooks to hanging bar */
        DEPLOY_HOOKS,

        /** Activate gas spring to pull robot up */
        HANG,

        PORTCULLIS_WAIT_FOR_HARDSTOP_CLEARANCE,

        BATTER_CHALLENGE,
    }

    /**
     * If there's a bug, revert to portcullis mode, it's the least likely to
     * break the robot
     */
    private static final SystemState PANIC_SYSTEM_STATE = SystemState.PORTCULLIS;
    private static final WantedState PANIC_WANTED_STATE = WantedState.PORTCULLIS;

    // Transitioning to portcullis mode is the least likely to destroy the
    // robot.
    private WantedState mWantedState = PANIC_WANTED_STATE;
    private boolean mIsAllowedToHang = false;
    private boolean mIsSafeToDriveThroughPortcullis = false;

    private Solenoid mArmLiftSolenoid = Constants.makeSolenoidForId(Constants.kArmLiftSolenoidId);
    private Solenoid mAdjustableHardStopSolenoid = Constants.makeSolenoidForId(Constants.kAdjustableHardStopSolenoidId);
    private Solenoid mCdfFlapSolenoid = Constants.makeSolenoidForId(Constants.kCdfFlapSolenoidId);
    private Solenoid mHookReleaseSolenoid = Constants.makeSolenoidForId(Constants.kHookReleaseSolenoidId);
    private Solenoid mGasSpringReleaseSolenoid = Constants.makeSolenoidForId(Constants.kGasSpringReleaseSolenoidId);

    Loop mLoop = new Loop() {

        private SystemState mSystemState = PANIC_SYSTEM_STATE;

        // Every time we transition states, we update the current state start
        // time and the state
        // changed boolean (for one cycle)
        private double mCurrentStateStartTime;

        @Override
        public void onStart() {
            // Leave the wanted state as it was set before enabling
            mSystemState = stateForOnStart();
            mCurrentStateStartTime = Timer.getFPGATimestamp();
            mIsAllowedToHang = false;
            mIsSafeToDriveThroughPortcullis = false;
        }

        @Override
        public void onLoop() {
            double now = Timer.getFPGATimestamp();
            SystemState newState = getNextState(now);
            if (newState != mSystemState) {
                System.out.println("Utility Arm state " + mSystemState + " to " + newState);
                mSystemState = newState;
                mCurrentStateStartTime = now;
            }
            setOutputsForState(mSystemState);
        }

        @Override
        public void onStop() {
        }

        private SystemState getNextState(double now) {
            double timeSinceStateStart = now - mCurrentStateStartTime;
            switch (mSystemState) {
            case SIZE_BOX:
                return handleSizeBox();
            case SIZE_BOX_TO_PORTCULLIS:
                return handleSizeBoxToPortcullis(timeSinceStateStart);
            case DRIVE_TO_PORTCULLIS:
                return handleDrivingToPortcullus(timeSinceStateStart);
            case PORTCULLIS:
                return handlePortcullis();
            case DRIVE:
                return handleDriving();
            case CDF:
                return handleCdf();
            case CDF_TO_DRIVING:
                return handleCdfToDriving(timeSinceStateStart);
            case LIFTING_ARM_FOR_HANG:
                return handleLiftingArmForHang(timeSinceStateStart);
            case OPENING_CDF_FOR_HANG:
                return handleOpeningCdfForHang(timeSinceStateStart);
            case DEPLOY_HOOKS:
                return handleDeployHooks();
            case HANG:
                return handleHang();
            case PORTCULLIS_WAIT_FOR_HARDSTOP_CLEARANCE:
                return handlePortcullisWaitForHardstopClearance(timeSinceStateStart);
            case BATTER_CHALLENGE:
                return handleBatterChallenge();
            default:
                System.out.println("Utility Arm unknown system state" + mSystemState);
                return PANIC_SYSTEM_STATE;
            }
        }
    };

    @Override
    public void outputToSmartDashboard() {
    }

    @Override
    public void stop() {
    }

    @Override
    public void zeroSensors() {
    }

    public Loop getLoop() {
        return mLoop;
    }

    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }

    public synchronized boolean isAllowedToHang() {
        return mIsAllowedToHang;
    }

    public synchronized boolean isSafeToDriveThroughPortcullis() {
        return mIsSafeToDriveThroughPortcullis;
    }

    /**
     * Picks an appropriate safe initial state for the system after the robot
     * has been re-enabled.
     * 
     * @return What the state should be as the looper starts.
     */
    private synchronized SystemState stateForOnStart() {
        switch (mWantedState) {
        case STAY_IN_SIZE_BOX:
            return SystemState.SIZE_BOX;
        case PORTCULLIS:
            return SystemState.PORTCULLIS;
        case DRIVING:
            return SystemState.DRIVE;
        case CDF:
            return SystemState.CDF;
        case PREPARE_FOR_HANG:
            return SystemState.OPENING_CDF_FOR_HANG;
        case PULL_UP_HANG:
            return SystemState.HANG;
        case BATTER_CHALLENGE:
            return SystemState.BATTER_CHALLENGE;
        default:
            System.out.println("UtilityArm Unknown wanted state: " + mWantedState);
            mWantedState = WantedState.PORTCULLIS;
            return SystemState.PORTCULLIS;
        }
    }

    private synchronized SystemState handleSizeBox() {
        return mWantedState == WantedState.STAY_IN_SIZE_BOX ? SystemState.SIZE_BOX : SystemState.SIZE_BOX_TO_PORTCULLIS;
    }

    private synchronized SystemState handleSizeBoxToPortcullis(double timeSinceStateStart) {
        if (mWantedState == WantedState.STAY_IN_SIZE_BOX) {
            logIllegalWantedState(SystemState.SIZE_BOX_TO_PORTCULLIS, mWantedState);
        }
        return timeSinceStateStart >= Constants.kUtilityArmSizeBoxToPortcullisDelay ? SystemState.PORTCULLIS
                : SystemState.SIZE_BOX_TO_PORTCULLIS;
    }

    private synchronized SystemState handleDrivingToPortcullus(double timeSinceStateStart) {
        switch (mWantedState) {
        case PREPARE_FOR_HANG: // fallthrough
        case BATTER_CHALLENGE: // fallthrough
        case PORTCULLIS:
            return timeSinceStateStart >= Constants.kUtilityArmDriveToPortcullisDelay ? SystemState.PORTCULLIS
                    : SystemState.DRIVE_TO_PORTCULLIS;
        case DRIVING: // fallthrough
        case CDF:
            return SystemState.DRIVE;
        case STAY_IN_SIZE_BOX: // fallthrough
        case PULL_UP_HANG: // fallthrough
        default:
            logIllegalWantedState(SystemState.DRIVE_TO_PORTCULLIS, mWantedState);
            return SystemState.DRIVE_TO_PORTCULLIS;
        }
    }

    private synchronized SystemState handlePortcullis() {
        mIsSafeToDriveThroughPortcullis = true;
        switch (mWantedState) {
        case PORTCULLIS:
            return SystemState.PORTCULLIS;
        case DRIVING: // fallthrough
        case CDF:
            mIsSafeToDriveThroughPortcullis = false;
            return SystemState.DRIVE;
        case PREPARE_FOR_HANG:
            mIsSafeToDriveThroughPortcullis = false;
            return SystemState.LIFTING_ARM_FOR_HANG; // TODO: should be wait for
                                                     // clearance
        case BATTER_CHALLENGE:
            mIsSafeToDriveThroughPortcullis = false;
            return SystemState.PORTCULLIS_WAIT_FOR_HARDSTOP_CLEARANCE;
        case STAY_IN_SIZE_BOX: // Fallthrough
        case PULL_UP_HANG: // Fallthrough
        default:
            logIllegalWantedState(SystemState.PORTCULLIS, mWantedState);
            return SystemState.PORTCULLIS;
        }
    }

    private synchronized SystemState handleDriving() {
        switch (mWantedState) {
        case PORTCULLIS: // fallthrough
        case BATTER_CHALLENGE: // fallthrough
        case PREPARE_FOR_HANG:
            return SystemState.DRIVE_TO_PORTCULLIS;
        case DRIVING:
            return SystemState.DRIVE;
        case CDF:
            return SystemState.CDF;
        case STAY_IN_SIZE_BOX: // fallthrough
        case PULL_UP_HANG: // fallthrough
        default:
            logIllegalWantedState(SystemState.DRIVE, mWantedState);
            return SystemState.DRIVE;
        }
    }

    private synchronized SystemState handleCdf() {
        switch (mWantedState) {
        case PORTCULLIS: // fallthrough
        case PREPARE_FOR_HANG: // fallthrough
        case BATTER_CHALLENGE: // fallthrough
        case DRIVING:
            return SystemState.CDF_TO_DRIVING;
        case CDF:
            return SystemState.CDF;
        case STAY_IN_SIZE_BOX: // fallthrough
        case PULL_UP_HANG: // fallthrough
        default:
            logIllegalWantedState(SystemState.CDF, mWantedState);
            return SystemState.CDF;
        }
    }

    private synchronized SystemState handleCdfToDriving(double timeSinceStateStart) {
        switch (mWantedState) {
        case PORTCULLIS: // fallthrough
        case PREPARE_FOR_HANG: // fallthrough
        case BATTER_CHALLENGE: // fallthrough
        case DRIVING:
            return timeSinceStateStart >= Constants.kUtilityArmCdfToDrivingDelay ? SystemState.DRIVE
                    : SystemState.CDF_TO_DRIVING;
        case CDF:
            return SystemState.CDF;
        case STAY_IN_SIZE_BOX: // fallthrough
        case PULL_UP_HANG: // fallthrough
        default:
            logIllegalWantedState(SystemState.CDF_TO_DRIVING, mWantedState);
            return SystemState.CDF_TO_DRIVING;
        }
    }

    private synchronized SystemState handleLiftingArmForHang(double timeSinceStateStart) {
        if (mWantedState != WantedState.PREPARE_FOR_HANG) {
            logIllegalWantedState(SystemState.LIFTING_ARM_FOR_HANG, mWantedState);
        }
        return timeSinceStateStart >= Constants.kUtilityArmLiftForHangToOpenCdfDelay ? SystemState.OPENING_CDF_FOR_HANG
                : SystemState.LIFTING_ARM_FOR_HANG;
    }

    private synchronized SystemState handleOpeningCdfForHang(double timeSinceStateStart) {
        if (mWantedState != WantedState.PREPARE_FOR_HANG) {
            logIllegalWantedState(SystemState.OPENING_CDF_FOR_HANG, mWantedState);
        }
        return timeSinceStateStart >= Constants.kUtilityArmOpenCdfToDeployHooksDelay ? SystemState.DEPLOY_HOOKS
                : SystemState.OPENING_CDF_FOR_HANG;
    }

    private synchronized SystemState handleDeployHooks() {
        mIsAllowedToHang = true;
        if (mWantedState == WantedState.PREPARE_FOR_HANG) {
            return SystemState.DEPLOY_HOOKS;
        } else if (mWantedState == WantedState.PULL_UP_HANG) {
            return SystemState.HANG;
        } else {
            logIllegalWantedState(SystemState.DEPLOY_HOOKS, mWantedState);
            return SystemState.DEPLOY_HOOKS;
        }
    }

    private synchronized SystemState handleHang() {
        if (mWantedState != WantedState.PULL_UP_HANG) {
            logIllegalWantedState(SystemState.HANG, mWantedState);
        }
        return SystemState.HANG;
    }

    private synchronized SystemState handleBatterChallenge() {
        switch (mWantedState) {
        case PORTCULLIS: // fallthrough
        case DRIVING: // fallthrough
        case CDF: // fallthrough
            // HACK: assuming size box drop delay is the same as batter
            // challenge delay
            return SystemState.SIZE_BOX_TO_PORTCULLIS;
        case PREPARE_FOR_HANG:
            return SystemState.LIFTING_ARM_FOR_HANG;
        case BATTER_CHALLENGE:
            return SystemState.BATTER_CHALLENGE;
        case PULL_UP_HANG: // fallthrough
        case STAY_IN_SIZE_BOX: // fallthrough
        default:
            logIllegalWantedState(SystemState.BATTER_CHALLENGE, mWantedState);
            return SystemState.BATTER_CHALLENGE;
        }
    }

    private synchronized SystemState handlePortcullisWaitForHardstopClearance(double timeSinceStateStart) {
        switch (mWantedState) {
        case PORTCULLIS: // fallthrough
        case DRIVING: // fallthrough
        case CDF: // fallthrough
            // HACK: assuming size box drop delay is the same as batter
            // challenge delay
            return SystemState.SIZE_BOX_TO_PORTCULLIS;
        case PREPARE_FOR_HANG:
            // TODO: stay in this state?
            return SystemState.LIFTING_ARM_FOR_HANG;
        case BATTER_CHALLENGE:
            return timeSinceStateStart >= Constants.kUtilityArmHardStopsMoveForRaiseArmDelay
                    ? SystemState.BATTER_CHALLENGE : SystemState.PORTCULLIS_WAIT_FOR_HARDSTOP_CLEARANCE;
        case PULL_UP_HANG: // fallthrough
        case STAY_IN_SIZE_BOX: // fallthrough
        default:
            logIllegalWantedState(SystemState.BATTER_CHALLENGE, mWantedState);
            return SystemState.PORTCULLIS_WAIT_FOR_HARDSTOP_CLEARANCE;
        }
    }

    private void logIllegalWantedState(SystemState systemState, WantedState wantedState) {
        System.out.println("Illegal Wanted state from " + systemState + " to " + wantedState);
    }

    private void setOutputsForState(SystemState systemState) {
        switch (systemState) {
        case SIZE_BOX_TO_PORTCULLIS: // fallthrough
        case DRIVE_TO_PORTCULLIS: // fallthrough
        case PORTCULLIS:
            setOutputs(ArmOutput.ARM_DOWN, AdjustableHardstopOutput.PREVENT_HANG, CdfFlapOutput.STOWED,
                    HookReleaseOutput.HOOKS_HELD_IN, GasSpringReleaseOutput.STOWED);
            break;
        case SIZE_BOX: // fallthrough
        case CDF_TO_DRIVING: // fallthrough
        case DRIVE:
            setOutputs(ArmOutput.ARM_UP, AdjustableHardstopOutput.PREVENT_HANG, CdfFlapOutput.STOWED,
                    HookReleaseOutput.HOOKS_HELD_IN, GasSpringReleaseOutput.STOWED);
            break;
        case CDF:
            setOutputs(ArmOutput.ARM_UP, AdjustableHardstopOutput.PREVENT_HANG, CdfFlapOutput.OPEN,
                    HookReleaseOutput.HOOKS_HELD_IN, GasSpringReleaseOutput.STOWED);
            break;
        case BATTER_CHALLENGE: // fallthrough
        case LIFTING_ARM_FOR_HANG:
            setOutputs(ArmOutput.ARM_UP, AdjustableHardstopOutput.ALLOW_HANG, CdfFlapOutput.STOWED,
                    HookReleaseOutput.HOOKS_HELD_IN, GasSpringReleaseOutput.STOWED);
            break;
        case OPENING_CDF_FOR_HANG:
            setOutputs(ArmOutput.ARM_UP, AdjustableHardstopOutput.ALLOW_HANG, CdfFlapOutput.OPEN,
                    HookReleaseOutput.HOOKS_HELD_IN, GasSpringReleaseOutput.STOWED);
            break;
        case DEPLOY_HOOKS:
            // TODO: add an intake up control here?
            setOutputs(ArmOutput.ARM_UP, AdjustableHardstopOutput.ALLOW_HANG, CdfFlapOutput.OPEN,
                    HookReleaseOutput.HOOKS_RELEASED, GasSpringReleaseOutput.STOWED);
            break;
        case HANG:
            // TODO: add an intake up control here?
            setOutputs(ArmOutput.ARM_UP, AdjustableHardstopOutput.ALLOW_HANG, CdfFlapOutput.OPEN,
                    HookReleaseOutput.HOOKS_RELEASED, GasSpringReleaseOutput.LIFTING_ROBOT);
            break;
        case PORTCULLIS_WAIT_FOR_HARDSTOP_CLEARANCE:
            setOutputs(ArmOutput.ARM_DOWN, AdjustableHardstopOutput.ALLOW_HANG, CdfFlapOutput.STOWED,
                    HookReleaseOutput.HOOKS_HELD_IN, GasSpringReleaseOutput.STOWED);
            break;
        default:
            System.out.println("Utility arm unknown state for output: " + systemState);
        }
    }

    /**
     * Always set all the outputs so we aren't tempted to merge states of the
     * solenoid outputs (which lose state on disable) and this state machine
     * (which keeps state through disable).
     */
    private void setOutputs(ArmOutput armOutput, AdjustableHardstopOutput adjustableHardstopOutput,
            CdfFlapOutput cdfFlapOutput, HookReleaseOutput hookReleaseOutput,
            GasSpringReleaseOutput gasSpringReleaseOutput) {
        mArmLiftSolenoid.set(armOutput.value);
        mAdjustableHardStopSolenoid.set(adjustableHardstopOutput.value);
        mCdfFlapSolenoid.set(cdfFlapOutput.value);
        mHookReleaseSolenoid.set(hookReleaseOutput.value);
        mGasSpringReleaseSolenoid.set(gasSpringReleaseOutput.value);
    }

    // These enums strongly type solenoid outputs to their respective solenoid
    // directions
    private enum ArmOutput {
        ARM_UP(false), ARM_DOWN(!ARM_UP.value);

        final boolean value;

        ArmOutput(boolean value) {
            this.value = value;
        }
    }

    private enum AdjustableHardstopOutput {
        PREVENT_HANG(false), ALLOW_HANG(!PREVENT_HANG.value);

        final boolean value;

        AdjustableHardstopOutput(boolean value) {
            this.value = value;
        }
    }

    private enum CdfFlapOutput {
        STOWED(false), OPEN(!STOWED.value);

        final boolean value;

        CdfFlapOutput(boolean value) {
            this.value = value;
        }
    }

    private enum HookReleaseOutput {
        HOOKS_HELD_IN(false), HOOKS_RELEASED(!HOOKS_HELD_IN.value);

        final boolean value;

        HookReleaseOutput(boolean value) {
            this.value = value;
        }
    }

    private enum GasSpringReleaseOutput {
        STOWED(false), LIFTING_ROBOT(!STOWED.value);

        final boolean value;

        GasSpringReleaseOutput(boolean value) {
            this.value = value;
        }
    }
}
