
package com.team254.frc2016;

import com.team254.frc2016.auto.AutoModeExecuter;
import com.team254.frc2016.loops.GyroCalibrator;
import com.team254.frc2016.loops.Looper;
import com.team254.frc2016.loops.RobotStateEstimator;
import com.team254.frc2016.loops.TurretResetter;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Intake;
import com.team254.frc2016.subsystems.Superstructure;
import com.team254.frc2016.subsystems.UtilityArm;
import com.team254.frc2016.vision.TargetInfo;
import com.team254.frc2016.vision.VisionServer;
import com.team254.frc2016.vision.VisionUpdate;
import com.team254.frc2016.vision.VisionUpdateReceiver;
import com.team254.lib.util.*;
import com.team254.logger.CheesyLogger;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
    private final CheesyLogger mCheesyLogger;

    // Subsystems
    Drive mDrive = Drive.getInstance();
    Superstructure mSuperstructure = Superstructure.getInstance();
    UtilityArm mUtilityArm = UtilityArm.getInstance();
    Compressor mCompressor = new Compressor(1);
    RevRoboticsAirPressureSensor mAirPressureSensor = new RevRoboticsAirPressureSensor(3);
    AutoModeExecuter mAutoModeExecuter = new AutoModeExecuter();

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

    boolean mGetDown = true;

    private LatchedBoolean mSelfieLatch = new LatchedBoolean();
    private LatchedBoolean mVisionLatch = new LatchedBoolean();

    public Robot() {
        mCheesyLogger = CheesyLogger.makeCheesyLogger("localhost");
    }

    public class TestReceiver implements VisionUpdateReceiver {

        @Override
        public void gotUpdate(VisionUpdate update) {
            mRobotState.addVisionUpdate(update.getCapturedAtTimestamp(), update.getTargets());
            for (int i = 0; i < update.getTargets().size(); i++) {
                TargetInfo target = update.getTargets().get(i);
                mCheesyLogger.sendTimePlotPoint("vision", "x", target.getX(), 1);
                mCheesyLogger.sendTimePlotPoint("vision", "y", target.getY(), 1);
                mCheesyLogger.sendTimePlotPoint("vision", "z", target.getZ(), 1);
                if (mLogToSmartdashboard) {
                    SmartDashboard.putNumber("goal_centroid_x", target.getX());
                    SmartDashboard.putNumber("goal_centroid_y", target.getY());
                    SmartDashboard.putNumber("goal_centroid_z", target.getZ());
                }
            }
        }
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
        mCheesyLogger.sendLogMessage("Robot Init-ed");
        mVisionServer.addVisionUpdateReceiver(new TestReceiver());

        // Reset all state
        zeroAllSensors();

        mUtilityArm.setWantedState(UtilityArm.WantedState.STAY_IN_SIZE_BOX);

        // Configure loopers
        mEnabledLooper.register(new TurretResetter());
        mEnabledLooper.register(RobotStateEstimator.getInstance());
        mEnabledLooper.register(Superstructure.getInstance().getLoop());
        mEnabledLooper.register(mDrive.getLoop());
        mEnabledLooper.register(mUtilityArm.getLoop());
        mDisabledLooper.register(new GyroCalibrator());

        mSmartDashboardInteractions.initWithDefaults();

        mCompressor.start();

        VisionServer.getInstance().setUseVisionMode();
    }

    @Override
    public void disabledInit() {
        mAutoModeExecuter.stop();
        mCheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.DISABLED);

        // Configure loopers
        mEnabledLooper.stop();
        mDisabledLooper.start();

        VisionServer.getInstance().setUseVisionMode();

        mDrive.setOpenLoop(DriveSignal.NEUTRAL);
        // Stop all actuators
        stopAll();
    }

    @Override
    public void autonomousInit() {
        // TODO: add option to force reset utility arm into starting box
        mAutoModeExecuter.stop();
        mCheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.AUTO);

        VisionServer.getInstance().setUseVisionMode();

        // Reset all sensors
        zeroAllSensors();

        // Shift to low
        mDrive.setHighGear(false);
        mSuperstructure.setTuningMode(false);

        maybeResetUtilityArmState();

        // Configure loopers
        mDisabledLooper.stop();
        mEnabledLooper.start();

        mAutoModeExecuter.setAutoMode(mSmartDashboardInteractions.getSelectedAutonMode());
        mAutoModeExecuter.start();
    }

    @Override
    public void teleopInit() {
        // TODO: add option to force reset utility arm into starting box
        mAutoModeExecuter.stop();
        mCheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.TELEOP);

        // Reset drive
        mDrive.resetEncoders();

        maybeResetUtilityArmState();

        // Configure loopers
        mDisabledLooper.stop();
        mEnabledLooper.start();
        mDrive.setOpenLoop(DriveSignal.NEUTRAL);

        VisionServer.getInstance().setUseVisionMode();

        mGetDown = true;
    }

    @Override
    public void disabledPeriodic() {
        // Keep kicking the CAN driver even though we are disabled...
        // See:
        // https://www.ctr-electronics.com/Talon%20SRX%20Software%20Reference%20Manual.pdf
        // page 130
        stopAll();

        outputAllToSmartDashboard();

        mHoodTuningMode = mSmartDashboardInteractions.isInHoodTuningMode();
        mLogToSmartdashboard = mSmartDashboardInteractions.shouldLogToSmartDashboard();
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

        if (mControls.getAutoAim()) {
            mSuperstructure.setWantedState(Superstructure.WantedState.WANT_TO_AIM);
            mGetDown = false;
        } else if (mControls.getBatterShot()) {
            mSuperstructure.setWantedState(Superstructure.WantedState.WANT_TO_BATTER);
            mGetDown = false;
        } else {
            mSuperstructure.setWantedState(
                    mGetDown ? Superstructure.WantedState.WANT_TO_STOW : Superstructure.WantedState.WANT_TO_IDLE);
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
            mUtilityArm.setWantedState(UtilityArm.WantedState.PORTCULLIS);
        } else if (mControls.getCdfButton()) {
            mUtilityArm.setWantedState(UtilityArm.WantedState.CDF);
        } else if (mControls.getBailButton()) {
            mUtilityArm.setWantedState(UtilityArm.WantedState.DRIVING);
        } else if (mControls.getBatterChallengeButton()) {
            mSuperstructure.stowIntake();
            mUtilityArm.setWantedState(UtilityArm.WantedState.BATTER_CHALLENGE);
        }

        boolean wantSelfieMode = false;

        if (mHoodTuningMode) {
            mSuperstructure.setTuningMode(true);
            if (mControls.getHoodTuningPositiveButton()) {
                mSuperstructure.setHoodManualScanOutput(0.1);
            } else if (mControls.getHoodTuningNegativeButton()) {
                mSuperstructure.setHoodManualScanOutput(-0.1);
            } else {
                mSuperstructure.setHoodManualScanOutput(0.0);
            }
        } else {
            mSuperstructure.setTuningMode(false);
            wantSelfieMode = mControls.getSelfieModeButton();
        }

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
    }

    @Override
    public void autonomousPeriodic() {
        outputAllToSmartDashboard();
    }

    private void maybeResetUtilityArmState() {
        if (mSmartDashboardInteractions.shouldResetUtilityArm()) {
            mUtilityArm.setWantedState(UtilityArm.WantedState.STAY_IN_SIZE_BOX);
        }
        mSmartDashboardInteractions.clearUtilityArmResetState();
    }
}
