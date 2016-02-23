package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Flywheel extends Subsystem {
    CANTalon master_talon_;
    CANTalon slave_talon_;

    Flywheel() {
        master_talon_ = new CANTalon(Constants.kShooterMasterId);
        slave_talon_ = new CANTalon(Constants.kShooterSlaveId);

        master_talon_.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (master_talon_.isSensorPresent(
                CANTalon.FeedbackDevice.CtreMagEncoder_Relative) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect shooter encoder!", false);
        }

        master_talon_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        slave_talon_.changeControlMode(CANTalon.TalonControlMode.Follower);
        slave_talon_.set(Constants.kShooterMasterId);

        master_talon_.setPID(Constants.kFlywheelKp, Constants.kFlywheelKi, Constants.kFlywheelKd, Constants.kFlywheelKf,
                Constants.kFlywheelIZone, Constants.kFlywheelRampRate, 0);
        master_talon_.setProfile(0);
        master_talon_.reverseSensor(false);
        master_talon_.reverseOutput(false);
        slave_talon_.reverseOutput(true);

        master_talon_.setVoltageRampRate(36.0);
        slave_talon_.setVoltageRampRate(36.0);

        master_talon_.enableBrakeMode(false);
        slave_talon_.enableBrakeMode(false);

        master_talon_.clearStickyFaults();
        slave_talon_.clearStickyFaults();
    }

    public synchronized double getRpm() {
        return master_talon_.getSpeed();
    }

    synchronized void setRpm(double rpm) {
        master_talon_.changeControlMode(CANTalon.TalonControlMode.Speed);
        master_talon_.set(rpm);
    }

    synchronized void setOpenLoop(double speed) {
        master_talon_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        master_talon_.set(speed);
    }

    public synchronized boolean isOnTarget() {
        return (master_talon_.getControlMode() == CANTalon.TalonControlMode.Speed
                && Math.abs(getRpm() - master_talon_.getSetpoint()) < Constants.kFlywheelOnTargetTolerance);
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("flywheel_rpm", getRpm());
        SmartDashboard.putNumber("flywheel_setpoint", master_talon_.getSetpoint());
        SmartDashboard.putBoolean("flywheel_on_target", isOnTarget());
    }

    @Override
    public void zeroSensors() {
        // no-op
    }
}
