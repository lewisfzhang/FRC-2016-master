package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Flywheel extends Subsystem {
    static Flywheel instance_ = new Flywheel();

    public static Flywheel getInstance() {
        return instance_;
    }

    CANTalon master_talon_;
    CANTalon slave_talon_;

    Flywheel() {
        master_talon_ = new CANTalon(Constants.kShooterMasterId);
        slave_talon_ = new CANTalon(Constants.kShooterSlaveId);

        master_talon_.enableBrakeMode(false);
        slave_talon_.enableBrakeMode(false);

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
        master_talon_.configNominalOutputVoltage(0, 0);
        master_talon_.configPeakOutputVoltage(12, 0);
        master_talon_.reverseSensor(false);
        master_talon_.reverseOutput(false);
        slave_talon_.reverseOutput(true);
    }

    public synchronized double getRpm() {
        return master_talon_.getSpeed();
    }

    public synchronized void setRpm(double rpm) {
        master_talon_.changeControlMode(CANTalon.TalonControlMode.Speed);
        master_talon_.set(rpm);
    }

    public synchronized void setOpenLoop(double speed) {
        master_talon_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        master_talon_.set(speed);
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("flywheel_rpm", getRpm());
        SmartDashboard.putNumber("flywheel_setpoint", master_talon_.getSetpoint());
    }

    @Override
    public void zeroSensors() {
        // no-op
    }
}
