
package com.team254.frc2016;

import com.team254.frc2016.auto.AutoModeExecuter;
import com.team254.frc2016.loops.GyroCalibrator;
import com.team254.frc2016.loops.Looper;
import com.team254.frc2016.loops.RobotStateEstimator;
import com.team254.frc2016.loops.TurretResetter;
import com.team254.frc2016.loops.VisionProcessor;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Superstructure;
import com.team254.frc2016.subsystems.UtilityArm;
import com.team254.frc2016.vision.VisionServer;
import com.team254.lib.util.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
    // Subsystems
    Drive mDrive = Drive.getInstance();
    Superstructure mSuperstructure = Superstructure.getInstance();
    UtilityArm mUtilityArm = UtilityArm.getInstance();
    Compressor mCompressor = new Compressor(1);
    RevRoboticsAirPressureSensor mAirPressureSensor = new RevRoboticsAirPressureSensor(3);
    AutoModeExecuter mAutoModeExecuter = null;
    Servo mDiddlerServo = new Servo(2);
    DigitalOutput mHasBallLightOutput = new DigitalOutput(0);

    // Other parts of the robot
    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    ControlBoard mControls = ControlBoard.getInstance();
    VisionServer mVisionServer = VisionServer.getInstance();
    RobotState mRobotState = RobotState.getInstance();

    // Enabled looper is called at 100Hz whenever the robot is enabled
    Looper mEnabledLooper = new Looper();
    // Disabled looper is called at 100Hz whenever the robot is disabled
    Looper mDisabledLooper = new Looper();

    SmartDashboardInteractions mSmartDashboardInteractions = new SmartDashboardInteractions();

    boolean mLogToSmartdashboard = true;
    boolean mHoodTuningMode = false;

    boolean mGetDown = false;

    private LatchedBoolean mSelfieLatch = new LatchedBoolean();
    private LatchedBoolean mVisionLatch = new LatchedBoolean();

    public Robot() {
    }

    public void stopAll() {
        mDrive.stop();
        mSuperstructure.stop();
        mUtilityArm.stop();
    }

    public void outputAllToSmartDashboard() {
        if (mLogToSmartdashboard) {
            mDrive.outputToSmartDashboard();
            mSuperstructure.outputToSmartDashboard();
            mRobotState.outputToSmartDashboard();
            mUtilityArm.outputToSmartDashboard();
            mEnabledLooper.outputToSmartDashboard();
        }
        // TODO: rate limit this
        SmartDashboard.putNumber("Air Pressure psi", mAirPressureSensor.getAirPressurePsi());
    }

    public void zeroAllSensors() {
        mDrive.zeroSensors();
        mSuperstructure.zeroSensors();
        mUtilityArm.zeroSensors();
        mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d(), new Rotation2d());
    }

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        mVisionServer.addVisionUpdateReceiver(VisionProcessor.getInstance());

        // Reset all state
        zeroAllSensors();

        mUtilityArm.setWantedState(UtilityArm.WantedState.STAY_IN_SIZE_BOX);

        // Configure loopers
        mEnabledLooper.register(new TurretResetter());
        mEnabledLooper.register(VisionProcessor.getInstance());
        mEnabledLooper.register(RobotStateEstimator.getInstance());
        mEnabledLooper.register(Superstructure.getInstance().getLoop());
        mEnabledLooper.register(mDrive.getLoop());
        mEnabledLooper.register(mUtilityArm.getLoop());
        mDisabledLooper.register(new GyroCalibrator());

        mSmartDashboardInteractions.initWithDefaults();

        mCompressor.start();
        mDiddlerServo.set(0);

        VisionServer.getInstance().setUseVisionMode();
    }

    @Override
    public void disabledInit() {
        if (mAutoModeExecuter != null) {
            mAutoModeExecuter.stop();
        }
        mAutoModeExecuter = null;

        // Configure loopers
        mEnabledLooper.stop();
        mDisabledLooper.start();

        VisionServer.getInstance().setUseVisionMode();

        mDrive.setOpenLoop(DriveSignal.NEUTRAL);
        mDrive.setBrakeMode(true);
        // Stop all actuators
        stopAll();
    }

    @Override
    public void autonomousInit() {
        if (mAutoModeExecuter != null) {
            mAutoModeExecuter.stop();
        }
        mAutoModeExecuter = null;

        VisionServer.getInstance().setUseVisionMode();

        // Reset all sensors
        zeroAllSensors();

        // Shift to high
        mDrive.setHighGear(true);
        mDrive.setBrakeMode(true);
        mSuperstructure.setTuningMode(false);

        maybeResetUtilityArmState();

        // Configure loopers
        mDisabledLooper.stop();
        mEnabledLooper.start();

        mAutoModeExecuter = new AutoModeExecuter();
        mAutoModeExecuter.setAutoMode(mSmartDashboardInteractions.getSelectedAutonMode());
        mAutoModeExecuter.start();

        mDiddlerServo.set(0);
    }

    @Override
    public void teleopInit() {
        if (mAutoModeExecuter != null) {
            mAutoModeExecuter.stop();
        }
        mAutoModeExecuter = null;

        // Reset drive
        mDrive.resetEncoders();

        maybeResetUtilityArmState();

        // Configure loopers
        mDisabledLooper.stop();
        mEnabledLooper.start();
        mDrive.setOpenLoop(DriveSignal.NEUTRAL);
        mDrive.setBrakeMode(false);

        VisionServer.getInstance().setUseVisionMode();

        mGetDown = false;
        mSuperstructure.setWantedState(Superstructure.WantedState.WANT_TO_DEPLOY);
        mSuperstructure.stowIntake();

        mDiddlerServo.set(0);
    }

    @Override
    public void disabledPeriodic() {
        // Keep kicking the CAN driver even though we are disabled...
        // See:
        // https://www.ctr-electronics.com/Talon%20SRX%20Software%20Reference%20Manual.pdf
        // page 130
        stopAll();

        mDrive.resetEncoders();

        outputAllToSmartDashboard();

        mHoodTuningMode = mSmartDashboardInteractions.isInHoodTuningMode();
        mLogToSmartdashboard = mSmartDashboardInteractions.shouldLogToSmartDashboard();
        mRobotState.reset(Timer.getFPGATimestamp(), new RigidTransform2d(), mSuperstructure.getTurret().getAngle());

        updateDriverFeedback();
    }

    @Override
    public void teleopPeriodic() {
        double throttle = mControls.getThrottle();
        double turn = mControls.getTurn();
        if (mControls.getTractionControl()) {
            Rotation2d heading_setpoint = mDrive.getGyroAngle();
            if (mDrive.getControlState() == Drive.DriveControlState.VELOCITY_HEADING_CONTROL) {
                heading_setpoint = mDrive.getVelocityHeadingSetpoint().getHeading();
            }
            mDrive.setVelocityHeadingSetpoint(
                    mCheesyDriveHelper.handleDeadband(throttle, CheesyDriveHelper.kThrottleDeadband)
                            * Constants.kDriveLowGearMaxSpeedInchesPerSec,
                    heading_setpoint);
        } else {
            mDrive.setBrakeMode(false);
            mDrive.setHighGear(!mControls.getLowGear());
            mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControls.getQuickTurn()));
        }

        if (mControls.getIntakeButton()) {
            mSuperstructure.deployIntake();
            mSuperstructure.setWantsToRunIntake();
        } else if (mControls.getStowIntakeButton()) {
            mSuperstructure.stowIntake();
            mSuperstructure.setWantsToStopIntake();
        } else if (mControls.getExhaustButton()) {
            mSuperstructure.setWantsToExhaust();
        } else {
            mSuperstructure.setWantsToStopIntake();
        }

        Superstructure.WantedState idle_state = mControls.getKeepWheelRunning()
                ? Superstructure.WantedState.WANT_TO_KEEP_SPINNING : Superstructure.WantedState.WANT_TO_DEPLOY;
        if (mControls.getAutoAim()) {
            mSuperstructure.setWantedState(Superstructure.WantedState.WANT_TO_AIM);
            mGetDown = false;
        } else if (mControls.getBatterShot()) {
            mSuperstructure.setWantedState(Superstructure.WantedState.WANT_TO_BATTER);
            mGetDown = false;
        } else if (mControls.getHoodUpButton()) {
            mSuperstructure.setWantedState(idle_state);
            mGetDown = false;
        } else if (mControls.getHoodDownButton()) {
            mSuperstructure.setWantedState(Superstructure.WantedState.WANT_TO_STOW);
            mGetDown = true;
        } else {
            mSuperstructure.setWantedState(mGetDown ? Superstructure.WantedState.WANT_TO_STOW : idle_state);
        }

        mSuperstructure.setTurretManualScanOutput(mControls.getTurretManual() * .66);

        if (mControls.getFireButton()) {
            mSuperstructure.setWantsToFireWhenReady();
        } else {
            mSuperstructure.setWantsToHoldFire();
        }

        if (mControls.getPortcullisButton()) {
            mSuperstructure.deployIntake();
            mGetDown = true;
            mUtilityArm.setWantedState(UtilityArm.WantedState.LOW_BAR);
        } else if (mControls.getCdfButton()) {
            mUtilityArm.setWantedState(UtilityArm.WantedState.LOW_BAR);
        } else if (mControls.getBailButton()) {
            mUtilityArm.setWantedState(UtilityArm.WantedState.DRIVING);
        } else if (mControls.getDeployHangerButton()) {
            mSuperstructure.stowIntake();
            mUtilityArm.setWantedState(UtilityArm.WantedState.PREPARE_FOR_HANG);
        }

        if (mUtilityArm.isAllowedToHang()) {
            if (mControls.getHang()) {
                mUtilityArm.setWantedState(UtilityArm.WantedState.PULL_UP_HANG);
            } else {
                mUtilityArm.setWantedState(UtilityArm.WantedState.PREPARE_FOR_HANG);
            }
        }

        if (mHoodTuningMode) {
            mSuperstructure.setTuningMode(true);
            if (mControls.getHoodTuningPositiveButton()) {
                mSuperstructure.setHoodManualScanOutput(0.05);
            } else if (mControls.getHoodTuningNegativeButton()) {
                mSuperstructure.setHoodManualScanOutput(-0.05);
            } else {
                mSuperstructure.setHoodManualScanOutput(0.0);
            }
        } else {
            mSuperstructure.setTuningMode(false);
        }

        boolean wantSelfieMode = false;
        if (mSelfieLatch.update(wantSelfieMode)) {
            VisionServer.getInstance().setUseIntakeMode();
        }
        if (mVisionLatch.update(!wantSelfieMode)) {
            VisionServer.getInstance().setUseVisionMode();
        }

        if (mControls.getHoodTuningPositiveButton()) {
            mSuperstructure.setTestServoSpeed(1.0);
        } else if (mControls.getHoodTuningNegativeButton()) {
            mSuperstructure.setTestServoSpeed(-1.0);
        } else {
            mSuperstructure.setTestServoSpeed(0.0);
        }

        outputAllToSmartDashboard();
        updateDriverFeedback();
    }

    @Override
    public void autonomousPeriodic() {
        outputAllToSmartDashboard();
        updateDriverFeedback();
    }

    private void maybeResetUtilityArmState() {
        if (mSmartDashboardInteractions.shouldResetUtilityArm()) {
            mUtilityArm.setWantedState(UtilityArm.WantedState.STAY_IN_SIZE_BOX);
        }
        mSmartDashboardInteractions.clearUtilityArmResetState();
    }

    private static final String COLOR_BOX_COLOR_KEY = "color_box_color";
    private static final String COLOR_BOX_TEXT_KEY = "color_box_text";

    private void updateDriverFeedback() {
        boolean hasBall = mSuperstructure.getIntake().hasBall();
        mHasBallLightOutput.set(hasBall);
        if (hasBall) {
            SmartDashboard.putString(COLOR_BOX_COLOR_KEY, "#00ff00");
            SmartDashboard.putString(COLOR_BOX_TEXT_KEY, "HAVE BALL");
        } else {
            SmartDashboard.putString(COLOR_BOX_COLOR_KEY, "#ff0000");
            SmartDashboard.putString(COLOR_BOX_TEXT_KEY, "NO BALL");
        }
    }
}
