package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;
import com.team254.frc2016.loops.Loop;
import com.team254.lib.util.ADXRS453_Gyro;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Rotation2d;

import com.team254.lib.util.SynchronousPID;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {
    protected static final int kVelocityControlSlot = 0;
    protected static final int kBaseLockControlSlot = 1;

    private static Drive instance_ = new Drive();
    private double mLastHeadingErrorDegrees;

    public static Drive getInstance() {
        return instance_;
    }

    public enum DriveControlState {
        OPEN_LOOP, BASE_LOCKED, VELOCITY_SETPOINT, VELOCITY_HEADING_CONTROL
    }

    private final CANTalon leftMaster_, leftSlave_, rightMaster_, rightSlave_;
    private final Solenoid shifter_;
    private final ADXRS453_Gyro gyro_;
    private DigitalInput lineSensor_;

    private DriveControlState driveControlState_;
    private VelocityHeadingSetpoint velocityHeadingSetpoint_;
    private SynchronousPID velocityHeadingPid_;

    private final Loop mLoop = new Loop() {
        @Override
        public void onStart() {
            setOpenLoop(DriveSignal.NEUTRAL);
        }

        @Override
        public void onLoop() {
            synchronized (Drive.this) {
                // System.out.println("State " + driveControlState_);
                switch (driveControlState_) {
                case OPEN_LOOP:
                    return;
                case BASE_LOCKED:
                    return;
                case VELOCITY_SETPOINT:
                    // Talons are updating the control loop state
                    return;
                case VELOCITY_HEADING_CONTROL:
                    updateVelocityHeadingSetpoint();
                    return;
                default:
                    System.out.println("WTF: unexpected drive control state: " + driveControlState_);
                    break;
                }
            }
        }

        @Override
        public void onStop() {
            setOpenLoop(DriveSignal.NEUTRAL);
        }
    };

    private Drive() {
        leftMaster_ = new CANTalon(Constants.kLeftDriveMasterId);
        leftSlave_ = new CANTalon(Constants.kLeftDriveSlaveId);
        rightMaster_ = new CANTalon(Constants.kRightDriveMasterId);
        rightSlave_ = new CANTalon(Constants.kRightDriveSlaveId);
        shifter_ = Constants.makeSolenoidForId(Constants.kShifterSolenoidId);
        shifter_.set(false); // high gear
        gyro_ = new ADXRS453_Gyro();
        lineSensor_ = new DigitalInput(Constants.kLineSensorDIO);

        // Get status at 100Hz
        leftMaster_.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
        rightMaster_.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);

        // Start in open loop mode
        leftMaster_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        leftMaster_.set(0);
        leftSlave_.changeControlMode(CANTalon.TalonControlMode.Follower);
        leftSlave_.set(Constants.kLeftDriveMasterId);
        rightMaster_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        rightMaster_.set(0);
        rightSlave_.changeControlMode(CANTalon.TalonControlMode.Follower);
        rightSlave_.set(Constants.kRightDriveMasterId);
        leftMaster_.enableBrakeMode(false);
        leftSlave_.enableBrakeMode(false);
        rightMaster_.enableBrakeMode(false);
        rightSlave_.enableBrakeMode(false);

        // Set up the encoders
        leftMaster_.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (leftMaster_.isSensorPresent(
                CANTalon.FeedbackDevice.CtreMagEncoder_Relative) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect left drive encoder!", false);
        }
        leftMaster_.reverseSensor(true);
        leftMaster_.reverseOutput(false);
        leftSlave_.reverseOutput(false);
        rightMaster_.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (rightMaster_.isSensorPresent(
                CANTalon.FeedbackDevice.CtreMagEncoder_Relative) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect right drive encoder!", false);
        }
        rightMaster_.reverseSensor(false);
        rightMaster_.reverseOutput(true);
        rightSlave_.reverseOutput(false);

        // Load velocity control gains
        leftMaster_.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
                Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
                kVelocityControlSlot);
        rightMaster_.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
                Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
                kVelocityControlSlot);
        // Load base lock control gains
        leftMaster_.setPID(Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd,
                Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate,
                kBaseLockControlSlot);
        rightMaster_.setPID(Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd,
                Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate,
                kBaseLockControlSlot);

        velocityHeadingPid_ = new SynchronousPID(Constants.kDriveHeadingVeloctyKp, Constants.kDriveHeadingVeloctyKi,
                Constants.kDriveHeadingVeloctyKd);
        velocityHeadingPid_.setOutputRange(-30, 30);

        setOpenLoop(DriveSignal.NEUTRAL);
    }

    public Loop getLoop() {
        return mLoop;
    }

    protected synchronized void setLeftRightPower(double left, double right) {
        leftMaster_.set(left);
        rightMaster_.set(-right);
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (driveControlState_ != DriveControlState.OPEN_LOOP) {
            leftMaster_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            leftMaster_.enableBrakeMode(signal.breakMode);
            leftSlave_.enableBrakeMode(signal.breakMode);
            rightMaster_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            rightMaster_.enableBrakeMode(signal.breakMode);
            rightSlave_.enableBrakeMode(signal.breakMode);
            driveControlState_ = DriveControlState.OPEN_LOOP;
        }
        setLeftRightPower(signal.leftMotor, signal.rightMotor);
    }

    public synchronized void setBaseLockOn() {
        if (driveControlState_ != DriveControlState.BASE_LOCKED) {
            leftMaster_.setProfile(kBaseLockControlSlot);
            leftMaster_.changeControlMode(CANTalon.TalonControlMode.Position);
            leftMaster_.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
            leftMaster_.enableBrakeMode(true);
            leftSlave_.enableBrakeMode(true);
            leftMaster_.set(leftMaster_.getPosition());
            rightMaster_.setProfile(kBaseLockControlSlot);
            rightMaster_.changeControlMode(CANTalon.TalonControlMode.Position);
            rightMaster_.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
            rightMaster_.enableBrakeMode(true);
            rightSlave_.enableBrakeMode(true);
            rightMaster_.set(rightMaster_.getPosition());
            driveControlState_ = DriveControlState.BASE_LOCKED;
        }
        setHighGear(false);
    }

    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        configureTalonsForSpeedControl();
        driveControlState_ = DriveControlState.VELOCITY_SETPOINT;
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    public synchronized void setVelocityHeadingSetpoint(double forward_inches_per_sec, Rotation2d headingSetpoint) {
        if (driveControlState_ != DriveControlState.VELOCITY_HEADING_CONTROL) {
            configureTalonsForSpeedControl();
            driveControlState_ = DriveControlState.VELOCITY_HEADING_CONTROL;
            velocityHeadingPid_.reset();
        }
        velocityHeadingSetpoint_ = new VelocityHeadingSetpoint(forward_inches_per_sec, headingSetpoint);
        updateVelocityHeadingSetpoint();
    }

    public double getLeftDistanceInches() {
        return rotationsToInches(leftMaster_.getPosition());
    }

    public double getRightDistanceInches() {
        return rotationsToInches(rightMaster_.getPosition());
    }

    public double getLeftVelocityInchesPerSec() {
        return rpmToInchesPerSecond(leftMaster_.getSpeed());
    }

    public double getRightVelocityInchesPerSec() {
        return rpmToInchesPerSecond(rightMaster_.getSpeed());
    }

    public ADXRS453_Gyro getGyro() {
        return gyro_;
    }

    public synchronized Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(gyro_.getAngle());
    }

    public void setHighGear(boolean high_gear) {
        shifter_.set(high_gear);
    }

    public synchronized void resetEncoders() {
        leftMaster_.setPosition(0);
        rightMaster_.setPosition(0);
    }

    public synchronized DriveControlState getControlState() {
        return driveControlState_;
    }

    public synchronized VelocityHeadingSetpoint getVelocityHeadingSetpoint() {
        return velocityHeadingSetpoint_;
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("left_distance", getLeftDistanceInches());
        SmartDashboard.putNumber("right_distance", getRightDistanceInches());
        SmartDashboard.putNumber("left_velocity", getLeftVelocityInchesPerSec());
        SmartDashboard.putNumber("right_velocity", getRightVelocityInchesPerSec());
        SmartDashboard.putNumber("gyro_angle", getGyro().getAngle());
        SmartDashboard.putNumber("gyro_center", getGyro().getCenter());
        SmartDashboard.putNumber("heading_error", mLastHeadingErrorDegrees);
    }

    @Override
    public synchronized void zeroSensors() {
        resetEncoders();
        gyro_.reset();
    }

    private void configureTalonsForSpeedControl() {
        if (driveControlState_ != DriveControlState.VELOCITY_HEADING_CONTROL
                && driveControlState_ != DriveControlState.VELOCITY_SETPOINT) {
            leftMaster_.changeControlMode(CANTalon.TalonControlMode.Speed);
            leftMaster_.setProfile(kVelocityControlSlot);
            leftMaster_.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
            leftMaster_.enableBrakeMode(true);
            leftSlave_.enableBrakeMode(true);
            setHighGear(false);
            rightMaster_.changeControlMode(CANTalon.TalonControlMode.Speed);
            rightMaster_.setProfile(kVelocityControlSlot);
            rightMaster_.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
            rightMaster_.enableBrakeMode(true);
            rightSlave_.enableBrakeMode(true);
            setHighGear(false);
        }
    }

    private synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (driveControlState_ == DriveControlState.VELOCITY_HEADING_CONTROL) {
            leftMaster_.set(inchesPerSecondToRpm(left_inches_per_sec));
            rightMaster_.set(inchesPerSecondToRpm(right_inches_per_sec));
        } else {
            System.out.println("Hit a bad velocity control state");
            leftMaster_.set(0);
            rightMaster_.set(0);
        }
    }

    private void updateVelocityHeadingSetpoint() {
        Rotation2d actualGyroAngle = getGyroAngle();

        mLastHeadingErrorDegrees = velocityHeadingSetpoint_.headingSetpoint_.rotateBy(actualGyroAngle.inverse())
                .getDegrees();

        double deltaSpeed = velocityHeadingPid_.calculate(mLastHeadingErrorDegrees);
        updateVelocitySetpoint(velocityHeadingSetpoint_.forwardInchesPerSec_ + deltaSpeed / 2,
                velocityHeadingSetpoint_.forwardInchesPerSec_ - deltaSpeed / 2);
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    public static class VelocityHeadingSetpoint {
        private final double forwardInchesPerSec_;
        private final Rotation2d headingSetpoint_;

        public VelocityHeadingSetpoint(double forwardInchesPerSec, Rotation2d headingSetpoint) {
            forwardInchesPerSec_ = forwardInchesPerSec;
            headingSetpoint_ = headingSetpoint;
        }

        public double getSpeed() {
            return this.forwardInchesPerSec_;
        }

        public Rotation2d getHeading() {
            return this.headingSetpoint_;
        }
    }
}
