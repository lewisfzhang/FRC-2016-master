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

        talon_.enableForwardSoftLimit(true);
        talon_.enableReverseSoftLimit(true);
        talon_.setForwardSoftLimit(Constants.kSoftMaxTurretAngle / (360.0 * Constants.kTurretRotationsPerTick));
        talon_.setReverseSoftLimit(Constants.kSoftMinTurretAngle / (360.0 * Constants.kTurretRotationsPerTick));
    }

    synchronized void setDesiredAngle(Rotation2d angle) {
        talon_.changeControlMode(CANTalon.TalonControlMode.Position);
        talon_.set(angle.getRadians() / (2 * Math.PI * Constants.kTurretRotationsPerTick));
    }

    synchronized void setOpenLoop(double speed) {
        talon_.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        talon_.set(speed);
    }

    synchronized void reset(Rotation2d actual_rotation) {
        talon_.setPosition(actual_rotation.getRadians() / (2 * Math.PI * Constants.kTurretRotationsPerTick));
    }

    public synchronized Rotation2d getAngle() {
        return Rotation2d.fromRadians(Constants.kTurretRotationsPerTick * talon_.getPosition() * 2 * Math.PI);
    }

    public synchronized boolean getForwardLimitSwitch() {
        return talon_.isFwdLimitSwitchClosed();
    }

    public synchronized boolean getReverseLimitSwitch() {
        return talon_.isRevLimitSwitchClosed();
    }

    public synchronized double getSetpoint() {
        return talon_.getSetpoint() * Constants.kTurretRotationsPerTick * 360.0;
    }

    private synchronized double getError() {
        return getAngle().getDegrees() - getSetpoint();
    }

    public synchronized boolean isOnTarget() {
        return (talon_.getControlMode() == CANTalon.TalonControlMode.Position
                && Math.abs(getError()) < Constants.kTurretOnTargetTolerance);
    }

    public synchronized boolean isSafe() {
        return (talon_.getControlMode() == CANTalon.TalonControlMode.Position && talon_.getSetpoint() == 0 && Math.abs(
                getAngle().getDegrees() * Constants.kTurretRotationsPerTick * 360.0) < Constants.kTurretSafeTolerance);
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("turret_error", getError());
        SmartDashboard.putNumber("turret_angle", getAngle().getDegrees());
        SmartDashboard.putNumber("turret_setpoint", getSetpoint());
        SmartDashboard.putBoolean("turret_fwd_limit", getForwardLimitSwitch());
        SmartDashboard.putBoolean("turret_rev_limit", getReverseLimitSwitch());
        SmartDashboard.putBoolean("turret_on_target", isOnTarget());
    }

    @Override
    public void zeroSensors() {
        reset(new Rotation2d());
    }
}
