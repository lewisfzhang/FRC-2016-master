
package com.team254.frc2016;

import com.team254.frc2016.loops.Looper;
import com.team254.frc2016.loops.RobotStateEstimator;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Flywheel;
import com.team254.frc2016.subsystems.Hood;
import com.team254.frc2016.subsystems.Intake;
import com.team254.frc2016.subsystems.Turret;
import com.team254.frc2016.vision.TargetInfo;
import com.team254.frc2016.vision.VisionServer;
import com.team254.frc2016.vision.VisionUpdate;
import com.team254.frc2016.vision.VisionUpdateReceiver;
import com.team254.lib.util.ADXRS453_Gyro;
import com.team254.lib.util.Pose2d;
import com.team254.lib.util.Rotation2d;
import com.team254.logger.CheesyLogger;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    private final CheesyLogger mCheesyLogger;
    Turret turret = Turret.getInstance();
    Drive drive = Drive.getInstance();
    Flywheel flywheel = Flywheel.getInstance();
    Intake intake = Intake.getInstance();
    Hood hood = Hood.getInstance();

    CheesyDriveHelper cdh = new CheesyDriveHelper();
    ControlBoard controls = ControlBoard.getInstance();
    VisionServer visionServer = VisionServer.getInstance();
    RobotState robot_state = RobotState.getInstance();

    Looper looper = Looper.getInstance();

    double gyroCalibrationStartTime = 0;

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
        drive.stop();
        intake.stop();
        hood.stop();
        turret.stop();
        flywheel.stop();
    }

    public void outputAllToSmartDashboard() {
        drive.outputToSmartDashboard();
        intake.outputToSmartDashboard();
        hood.outputToSmartDashboard();
        turret.outputToSmartDashboard();
        flywheel.outputToSmartDashboard();
    }

    public void zeroAllSensors() {
        drive.zeroSensors();
        intake.zeroSensors();
        hood.zeroSensors();
        turret.zeroSensors();
        flywheel.zeroSensors();
    }

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        mCheesyLogger.sendLogMessage("Robot Init-ed");
        visionServer.addVisionUpdateReceiver(new TestReceiver());

        zeroAllSensors();
        looper.register(RobotStateEstimator.getInstance());
        robot_state.reset(System.nanoTime(), new Pose2d(), new Rotation2d());
    }

    @Override
    public void disabledInit() {
        mCheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.DISABLED);
        looper.stop();
        stopAll();
    }

    @Override
    public void autonomousInit() {
        mCheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.AUTO);
        robot_state.reset(System.nanoTime(), new Pose2d(), new Rotation2d());
        drive.getGyro().cancelCalibrate();
        drive.zeroSensors();
        looper.start();
    }

    @Override
    public void teleopInit() {
        mCheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.TELEOP);
        robot_state.reset(System.nanoTime(), new Pose2d(), new Rotation2d());
        drive.getGyro().cancelCalibrate();
        drive.resetEncoders();
        looper.start();
    }

    @Override
    public void disabledPeriodic() {
        double now = Timer.getFPGATimestamp();
        // Keep re-calibrating the gyro every 5 seconds
        if (now - gyroCalibrationStartTime > ADXRS453_Gyro.kCalibrationSampleTime) {
            drive.getGyro().endCalibrate();
            gyroCalibrationStartTime = now;
            drive.getGyro().startCalibrate();
        }
        if (controls.getQuickTurn()) {
            turret.reset(new Rotation2d());
        }

        outputAllToSmartDashboard();
    }

    @Override
    public void teleopPeriodic() {
        double throttle = controls.getThrottle();
        double turn = controls.getTurn();
        if (controls.getBaseLock()) {
            // drive.baseLock();
        } else {
            // drive.setOpenLoop(cdh.cheesyDrive(throttle, turn,
            // controls.getQuickTurn()));
        }

        // turret.setDesiredAngle(Rotation2d.fromDegrees(180 * turn));
        flywheel.setOpenLoop(throttle);
        // test_servo.set(throttle);
        // test_servo2.set(-throttle);
        // intake.set(throttle);

        outputAllToSmartDashboard();
    }
}
