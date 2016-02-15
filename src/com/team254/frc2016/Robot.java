
package com.team254.frc2016;

import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Flywheel;
import com.team254.frc2016.subsystems.Intake;
import com.team254.frc2016.subsystems.Turret;
import com.team254.frc2016.vision.TargetInfo;
import com.team254.frc2016.vision.VisionServer;
import com.team254.frc2016.vision.VisionUpdate;
import com.team254.frc2016.vision.VisionUpdateReceiver;
import com.team254.lib.util.ADXRS453_Gyro;
import com.team254.lib.util.Rotation2d;
import com.team254.logger.CheesyLogger;

import edu.wpi.first.wpilibj.ContinuousRotationServo;
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
    CheesyDriveHelper cdh = new CheesyDriveHelper();
    ControlBoard controls = ControlBoard.getInstance();
    VisionServer visionServer = VisionServer.getInstance();

    // MA3Encoder test_encoder = new MA3Encoder(0);
    ContinuousRotationServo test_servo = new ContinuousRotationServo(0);
    ContinuousRotationServo test_servo2 = new ContinuousRotationServo(1);

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

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        mCheesyLogger.sendLogMessage("Robot Init-ed");
        visionServer.addVisionUpdateReceiver(new TestReceiver());
    }

    @Override
    public void disabledInit() {
        mCheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.DISABLED);
        drive.stop();
    }

    @Override
    public void autonomousInit() {
        mCheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.AUTO);
        drive.getGyro().cancelCalibrate();
        drive.getGyro().reset();
    }

    @Override
    public void teleopInit() {
        mCheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.TELEOP);
        drive.getGyro().cancelCalibrate();
        drive.resetEncoders();
    }

    int count = 0;

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
        // System.out.println("MA3 angle " +
        // test_encoder.getContinuousAngleDegrees());
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

        mCheesyLogger.sendTimePlotPoint("joystick", "throttle", throttle, 100);
        mCheesyLogger.sendTimePlotPoint("joystick", "turn", turn, 100);

        //turret.setDesiredAngle(Rotation2d.fromDegrees(180 * turn));
        //flywheel.setOpenLoop(throttle);
        test_servo.set(throttle);
        test_servo2.set(-throttle);
        //intake.set(throttle);

        SmartDashboard.putNumber("flywheel_rpm", flywheel.getRpm());
        SmartDashboard.putNumber("turret_angle", turret.getAngle().getDegrees());
        SmartDashboard.putBoolean("turret_fwd_limit", turret.getForwardLimitSwitch());
        SmartDashboard.putBoolean("turret_rev_limit", turret.getReverseLimitSwitch());
        SmartDashboard.putNumber("left_distance", drive.getLeftDistanceInches());
        SmartDashboard.putNumber("right_distance", drive.getRightDistanceInches());
        SmartDashboard.putNumber("left_velocity", drive.getLeftVelocityInchesPerSec());
        SmartDashboard.putNumber("right_velocity", drive.getRightVelocityInchesPerSec());
        SmartDashboard.putNumber("gyro_angle", drive.getGyro().getAngle());
    }
}
