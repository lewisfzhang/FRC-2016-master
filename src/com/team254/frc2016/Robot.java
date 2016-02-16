
package com.team254.frc2016;

import com.team254.frc2016.loops.GyroCalibrator;
import com.team254.frc2016.loops.Looper;
import com.team254.frc2016.loops.RobotStateEstimator;
import com.team254.frc2016.loops.TurretResetter;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Flywheel;
import com.team254.frc2016.subsystems.Hood;
import com.team254.frc2016.subsystems.Intake;
import com.team254.frc2016.subsystems.Turret;
import com.team254.frc2016.vision.TargetInfo;
import com.team254.frc2016.vision.VisionServer;
import com.team254.frc2016.vision.VisionUpdate;
import com.team254.frc2016.vision.VisionUpdateReceiver;
import com.team254.lib.util.Pose2d;
import com.team254.lib.util.Rotation2d;
import com.team254.logger.CheesyLogger;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
    private final CheesyLogger mCheesyLogger;

    // Subsystems
    Turret mTurret = Turret.getInstance();
    Drive mDrive = Drive.getInstance();
    Flywheel mFlywheel = Flywheel.getInstance();
    Intake mIntake = Intake.getInstance();
    Hood mHood = Hood.getInstance();

    // Other parts of the robot
    CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    ControlBoard mControls = ControlBoard.getInstance();
    VisionServer mVisionServer = VisionServer.getInstance();
    RobotState mRobotState = RobotState.getInstance();

    // Enabled looper is called at 100Hz whenever the robot is enabled
    Looper mEnabledLooper = new Looper();
    // Disabled looper is called at 100Hz whenever the robot is disabled
    Looper mDisabledLooper = new Looper();

    public Robot() {
        mCheesyLogger = CheesyLogger.makeCheesyLogger("10.2.54.195");
    }

    public class TestReceiver implements VisionUpdateReceiver {

        @Override
        public void gotUpdate(VisionUpdate update) {
            for (int i = 0; i < update.getTargets().size(); i++) {
                TargetInfo target = update.getTargets().get(i);
                // System.out.println(i + " : " + target.getAngle().getDegrees()
                // + " : " + target.getDistance());
                mCheesyLogger.sendTimePlotPoint("vision_angle", "angle", target.getAngle().getDegrees(), 20);
                mCheesyLogger.sendTimePlotPoint("vision_distance", "distance", target.getDistance(), 20);
                SmartDashboard.putNumber("Angle", target.getAngle().getDegrees());
                SmartDashboard.putNumber("Distance", target.getDistance());
            }

        }
    }

    public void stopAll() {
        mDrive.stop();
        mIntake.stop();
        mHood.stop();
        mTurret.stop();
        mFlywheel.stop();
    }

    public void outputAllToSmartDashboard() {
        mDrive.outputToSmartDashboard();
        mIntake.outputToSmartDashboard();
        mHood.outputToSmartDashboard();
        mTurret.outputToSmartDashboard();
        mFlywheel.outputToSmartDashboard();

        mRobotState.outputToSmartDashboard();
    }

    public void zeroAllSensors() {
        mDrive.zeroSensors();
        mIntake.zeroSensors();
        mHood.zeroSensors();
        mTurret.zeroSensors();
        mFlywheel.zeroSensors();
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
        mEnabledLooper.register(mHood.getLoop());
        mDisabledLooper.register(new GyroCalibrator());
    }

    @Override
    public void disabledInit() {
        mCheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.DISABLED);

        // Configure loopers
        mEnabledLooper.stop();
        mDisabledLooper.start();

        // Stop all actuators
        stopAll();
    }

    @Override
    public void autonomousInit() {
        mCheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.AUTO);

        // Reset all state
        zeroAllSensors();

        // Configure loopers
        mDisabledLooper.stop();
        mEnabledLooper.start();
    }

    @Override
    public void teleopInit() {
        mCheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.TELEOP);

        // Reset drive
        mDrive.resetEncoders();

        // Configure loopers
        mDisabledLooper.stop();
        mEnabledLooper.start();
    }

    @Override
    public void disabledPeriodic() {
        if (mControls.getQuickTurn()) {
            mTurret.reset(new Rotation2d());
        }

        outputAllToSmartDashboard();
    }

    @Override
    public void teleopPeriodic() {
        double throttle = mControls.getThrottle();
        double turn = mControls.getTurn();
        if (mControls.getBaseLock()) {
            // drive.baseLock();
        } else {
            // drive.setOpenLoop(cdh.cheesyDrive(throttle, turn,
            // controls.getQuickTurn()));
        }

        // turret.setDesiredAngle(Rotation2d.fromDegrees(180 * turn));
        mFlywheel.setOpenLoop(throttle);
        // test_servo.set(throttle);
        // test_servo2.set(-throttle);
        // intake.set(throttle);

        outputAllToSmartDashboard();
    }
}
