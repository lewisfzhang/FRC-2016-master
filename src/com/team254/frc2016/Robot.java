
package com.team254.frc2016;

import com.team254.frc2016.auto.AutoModeExecuter;
import com.team254.frc2016.auto.modes.OneBallMode;
import com.team254.frc2016.loops.GyroCalibrator;
import com.team254.frc2016.loops.Looper;
import com.team254.frc2016.loops.RobotStateEstimator;
import com.team254.frc2016.loops.TurretResetter;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Intake;
import com.team254.frc2016.subsystems.Shooter;
import com.team254.frc2016.vision.TargetInfo;
import com.team254.frc2016.vision.VisionServer;
import com.team254.frc2016.vision.VisionUpdate;
import com.team254.frc2016.vision.VisionUpdateReceiver;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Pose2d;
import com.team254.lib.util.Rotation2d;
import com.team254.logger.CheesyLogger;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
    private final CheesyLogger mCheesyLogger;

    // Subsystems
    Drive mDrive = Drive.getInstance();
    Intake mIntake = Intake.getInstance();
    Shooter mShooter = Shooter.getInstance();
    Compressor mCompressor = new Compressor(1);
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
    
    //NetworkTables
    public static NetworkTable shooterTable = NetworkTable.getTable("shooter");

    public Robot() {
        mCheesyLogger = CheesyLogger.makeCheesyLogger("localhost");
    }

    public class TestReceiver implements VisionUpdateReceiver {

        @Override
        public void gotUpdate(VisionUpdate update) {
            mRobotState.addVisionUpdate(update.getCapturedAtTimestamp(), update.getTargets());
            for (int i = 0; i < update.getTargets().size(); i++) {
                TargetInfo target = update.getTargets().get(i);
                // System.out.println(i + " : " + target.getAngle().getDegrees()
                // + " : " + target.getDistance());
                mCheesyLogger.sendTimePlotPoint("vision", "x", target.getX(), 1);
                mCheesyLogger.sendTimePlotPoint("vision", "y", target.getY(), 1);
                mCheesyLogger.sendTimePlotPoint("vision", "z", target.getZ(), 1);
                SmartDashboard.putNumber("goal_centroid_x", target.getX());
                SmartDashboard.putNumber("goal_centroid_y", target.getY());
            }

        }
    }

    public void stopAll() {
        mDrive.stop();
        mIntake.stop();
        mShooter.stop();
    }

    public void outputAllToSmartDashboard() {
        mDrive.outputToSmartDashboard();
        mIntake.outputToSmartDashboard();
        mShooter.outputToSmartDashboard();

        mRobotState.outputToSmartDashboard();
    }

    public void zeroAllSensors() {
        mDrive.zeroSensors();
        mIntake.zeroSensors();
        mShooter.zeroSensors();
        mRobotState.reset(Timer.getFPGATimestamp(), new Pose2d(), new Rotation2d());
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

        // Configure loopers
        mEnabledLooper.register(new TurretResetter());
        mEnabledLooper.register(RobotStateEstimator.getInstance());
        mEnabledLooper.register(Shooter.getInstance().getLoop());
        mEnabledLooper.register(mDrive.getLoop());
        mDisabledLooper.register(new GyroCalibrator());

        mCompressor.start();
    }

    @Override
    public void disabledInit() {
        mAutoModeExecuter.stop();
        mCheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.DISABLED);

        // Configure loopers
        mEnabledLooper.stop();
        mDisabledLooper.start();

        mDrive.setOpenLoop(DriveSignal.NEUTRAL);
        // Stop all actuators
        stopAll();
    }

    @Override
    public void autonomousInit() {
        mAutoModeExecuter.stop();
        mCheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.AUTO);

        // Reset all state
        zeroAllSensors();

        // Configure loopers
        mDisabledLooper.stop();
        mEnabledLooper.start();

        mAutoModeExecuter.setAutoMode(new OneBallMode(mDrive, mShooter));
        mAutoModeExecuter.start();
    }

    @Override
    public void teleopInit() {
        mAutoModeExecuter.stop();
        mCheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.TELEOP);

        // Reset drive
        mDrive.resetEncoders();

        // Configure loopers
        mDisabledLooper.stop();
        mEnabledLooper.start();
        mDrive.setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void disabledPeriodic() {
        if (mControls.getQuickTurn()) {
            mShooter.zeroTurret();
        }

        outputAllToSmartDashboard();
    }

    @Override
    public void teleopPeriodic() {
        double throttle = mControls.getThrottle();
        double turn = mControls.getTurn();
        if (mControls.getBaseLock()) {
            mDrive.setBaseLockOn();
        } else {
            mDrive.setHighGear(!mControls.getLowGear());
            mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControls.getQuickTurn()));
        }

        if (mControls.getIntakeButton()) {
            mIntake.deploy(true);
            mIntake.set(1.0);
        } else if (mControls.getStowIntakeButton()) {
            mIntake.deploy(false);
            mIntake.set(0.0);
        } else if (mControls.getExhaustButton()) {
            mIntake.set(-1.0);
        } else {
            mIntake.set(0.0);
        }

        if (mControls.getAutoAim()) {
            mShooter.wantAutoAim();
        } else if (mControls.getBatterShot()) {
            mShooter.wantBatterMode();
        } else {
            mShooter.wantStow();
        }

        mShooter.moveTurretManual(mControls.getTurretManual());

        if (mControls.getFireButton()) {
            mShooter.wantShootNow();
        }

        // TODO: Remove these before competition
        if (mControls.getButton5()) {
            mShooter.setHoodBias(mShooter.getHoodBias() + .1);
        } else if (mControls.getButton6()) {
            mShooter.setHoodBias(mShooter.getHoodBias() - .1);
        } else if (mControls.getButton4()) {
            mShooter.setHoodBias(0);
        }

        outputAllToSmartDashboard();
    }

    @Override
    public void autonomousPeriodic() {
        outputAllToSmartDashboard();
    }
}
