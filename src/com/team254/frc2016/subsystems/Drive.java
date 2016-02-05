package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;
import com.team254.lib.util.ADXRS453_Gyro;
import com.team254.lib.util.DriveSignal;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;

public class Drive {
    protected static final int kVelocityControlSlot = 0;
    protected static final int kBaseLockControlSlot = 1;

    private static Drive instance_ = new Drive();

    public static Drive getInstance() {
        return instance_;
    }

    private final CANTalon leftMaster_, leftSlave_, rightMaster_, rightSlave_;
    private final Solenoid shifter_;
    private final ADXRS453_Gyro gyro_;

    private Drive() {
        leftMaster_ = new CANTalon(Constants.kLeftDriveMasterId);
        leftSlave_ = new CANTalon(Constants.kLeftDriveSlaveId);
        rightMaster_ = new CANTalon(Constants.kRightDriveMasterId);
        rightSlave_ = new CANTalon(Constants.kRightDriveSlaveId);
        shifter_ = new Solenoid(Constants.kShifterSolenoidId);
        shifter_.set(false); // low gear
        gyro_ = new ADXRS453_Gyro();

        // Get status at 100Hz
        leftMaster_.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
        rightMaster_.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);

        // Start in open loop mode
        leftMaster_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        leftMaster_.set(0);
        leftSlave_.changeControlMode(CANTalon.TalonControlMode.Follower);
        leftMaster_.set(Constants.kLeftDriveMasterId);
        rightMaster_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        rightMaster_.set(0);
        rightSlave_.changeControlMode(CANTalon.TalonControlMode.Follower);
        rightSlave_.set(Constants.kRightDriveMasterId);
        leftMaster_.enableBrakeMode(false);
        leftSlave_.enableBrakeMode(false);
        rightMaster_.enableBrakeMode(false);
        rightSlave_.enableBrakeMode(false);

        leftMaster_.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (leftMaster_.isSensorPresent(
                CANTalon.FeedbackDevice.CtreMagEncoder_Relative) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect left drive encoder!", false);
        }
        rightMaster_.reverseSensor(false);
        rightMaster_.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (rightMaster_.isSensorPresent(
                CANTalon.FeedbackDevice.CtreMagEncoder_Relative) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect right drive encoder!", false);
        }
        rightMaster_.reverseSensor(true);

        // Load velocity control gains
        leftMaster_.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
                Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
                kVelocityControlSlot);
        rightMaster_.setPID(Constants.kDriveVelocityKp, Constants.kDriveVelocityKi, Constants.kDriveVelocityKd,
                Constants.kDriveVelocityKf, Constants.kDriveVelocityIZone, Constants.kDriveVelocityRampRate,
                kVelocityControlSlot);
        leftMaster_.setPID(Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd,
                Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate,
                kBaseLockControlSlot);
        rightMaster_.setPID(Constants.kDriveBaseLockKp, Constants.kDriveBaseLockKi, Constants.kDriveBaseLockKd,
                Constants.kDriveBaseLockKf, Constants.kDriveBaseLockIZone, Constants.kDriveBaseLockRampRate,
                kBaseLockControlSlot);
    }

    protected synchronized void setLeftRightPower(double left, double right) {
        leftMaster_.set(left);
        rightMaster_.set(-right);
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (leftMaster_.getControlMode() != CANTalon.TalonControlMode.PercentVbus) {
            leftMaster_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            leftMaster_.enableBrakeMode(false);
            leftSlave_.enableBrakeMode(false);
        }
        if (rightMaster_.getControlMode() != CANTalon.TalonControlMode.PercentVbus) {
            rightMaster_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
            rightMaster_.enableBrakeMode(false);
            rightSlave_.enableBrakeMode(false);
        }
        setLeftRightPower(signal.leftMotor, signal.rightMotor);
    }

    private double encoderCountsToInches(double counts) {
        return counts * (Constants.kDriveWheelDiameterInches * Math.PI) / 4096;
    }

    private double inchesToEncoderCounts(double inches) {
        return inches * 4096 / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private double encoderVelocityToInchesPerSec(double counts) {
        return encoderCountsToInches(counts) * 10.0;
    }

    private double inchesPerSecToEncoderVelocity(double inches_per_sec) {
        return (int) (inchesToEncoderCounts(inches_per_sec) / 10.0);
    }

    public double getLeftDistanceInches() {
        return encoderCountsToInches(leftMaster_.getPosition());
    }

    public double getRightDistanceInches() {
        return encoderCountsToInches(rightMaster_.getPosition());
    }

    public double getLeftVelocityInchesPerSec() {
        return encoderVelocityToInchesPerSec(leftMaster_.getSpeed());
    }

    public double getRightVelocityInchesPerSec() {
        return encoderVelocityToInchesPerSec(rightMaster_.getSpeed());
    }

    public ADXRS453_Gyro getGyro() {
        return gyro_;
    }

    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    protected synchronized void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        leftMaster_.set(inchesPerSecToEncoderVelocity(left_inches_per_sec));
        rightMaster_.set(inchesPerSecToEncoderVelocity(right_inches_per_sec));
    }

    public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
        if (leftMaster_.getControlMode() != CANTalon.TalonControlMode.Speed) {
            leftMaster_.changeControlMode(CANTalon.TalonControlMode.Speed);
            leftMaster_.setProfile(kVelocityControlSlot);
            leftMaster_.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
            leftMaster_.enableBrakeMode(true);
            leftSlave_.enableBrakeMode(true);
            setHighGear(true);
        }
        if (rightMaster_.getControlMode() != CANTalon.TalonControlMode.Speed) {
            rightMaster_.changeControlMode(CANTalon.TalonControlMode.Speed);
            rightMaster_.setProfile(kVelocityControlSlot);
            rightMaster_.setAllowableClosedLoopErr(Constants.kDriveVelocityAllowableError);
            rightMaster_.enableBrakeMode(true);
            rightSlave_.enableBrakeMode(true);
            setHighGear(true);
        }
        updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
    }

    public synchronized void baseLock() {
        if (leftMaster_.getControlMode() != CANTalon.TalonControlMode.Position) {
            leftMaster_.changeControlMode(CANTalon.TalonControlMode.Position);
            leftMaster_.setProfile(kBaseLockControlSlot);
            leftMaster_.set(leftMaster_.getEncPosition());
            leftMaster_.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
            leftMaster_.enableBrakeMode(true);
            leftSlave_.enableBrakeMode(true);
            setHighGear(false);
        }
        if (rightMaster_.getControlMode() != CANTalon.TalonControlMode.Position) {
            rightMaster_.changeControlMode(CANTalon.TalonControlMode.Position);
            rightMaster_.setProfile(kBaseLockControlSlot);
            rightMaster_.set(rightMaster_.getEncPosition());
            rightMaster_.setAllowableClosedLoopErr(Constants.kDriveBaseLockAllowableError);
            rightMaster_.enableBrakeMode(true);
            rightSlave_.enableBrakeMode(true);
            setHighGear(false);
        }
    }

    public void setHighGear(boolean high_gear) {
        shifter_.set(high_gear);
    }
}
