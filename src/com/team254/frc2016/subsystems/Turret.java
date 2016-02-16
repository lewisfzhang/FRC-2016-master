package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;
import com.team254.lib.util.Rotation2d;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends Subsystem {
    private CANTalon talon_;

    Turret() {
        talon_ = new CANTalon(Constants.kTurretTalonId);
        talon_.enableBrakeMode(true);
        talon_.enableLimitSwitch(true, true);
        talon_.ConfigFwdLimitSwitchNormallyOpen(true);
        talon_.ConfigRevLimitSwitchNormallyOpen(true);
        talon_.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
        talon_.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (talon_.isSensorPresent(
                CANTalon.FeedbackDevice.CtreMagEncoder_Relative) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect turret encoder!", false);
        }

        talon_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);

        talon_.setPID(Constants.kTurretKp, Constants.kTurretKi, Constants.kTurretKd, Constants.kTurretKf,
                Constants.kTurretIZone, Constants.kTurretRampRate, 0);
        talon_.setProfile(0);
        talon_.reverseSensor(true);
        talon_.reverseOutput(false);
    }

    synchronized void setDesiredAngle(Rotation2d angle) {
        talon_.changeControlMode(CANTalon.TalonControlMode.Position);
        talon_.set(angle.getRadians() / (2 * Math.PI * Constants.kTurretGearReduction));
    }

    synchronized void setOpenLoop(double speed) {
        talon_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        talon_.set(speed);
    }

    synchronized void reset(Rotation2d actual_rotation) {
        talon_.setPosition(actual_rotation.getRadians() / (2 * Math.PI * Constants.kTurretGearReduction));
    }

    public synchronized Rotation2d getAngle() {
        return Rotation2d.fromRadians(Constants.kTurretGearReduction * talon_.getPosition() * 2 * Math.PI);
    }

    public synchronized boolean getForwardLimitSwitch() {
        return talon_.isFwdLimitSwitchClosed();
    }

    public synchronized boolean getReverseLimitSwitch() {
        return talon_.isRevLimitSwitchClosed();
    }

    public synchronized boolean isOnTarget() {
        return (talon_.getControlMode() == CANTalon.TalonControlMode.Position && Math.abs(
                Constants.kTurretGearReduction * talon_.getError() * 2 * Math.PI) < Constants.kTurretOnTargetTolerance);
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("turret_angle", getAngle().getDegrees());
        SmartDashboard.putNumber("turret_setpoint", talon_.getSetpoint() / (360.0 * Constants.kTurretGearReduction));
        SmartDashboard.putBoolean("turret_fwd_limit", getForwardLimitSwitch());
        SmartDashboard.putBoolean("turret_rev_limit", getReverseLimitSwitch());
        SmartDashboard.putBoolean("turret_on_target", isOnTarget());
    }

    @Override
    public void zeroSensors() {
        reset(new Rotation2d());
    }
}
