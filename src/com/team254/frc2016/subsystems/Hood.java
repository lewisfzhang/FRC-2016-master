package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;
import com.team254.frc2016.loops.Loop;
import com.team254.lib.util.MA3AnalogEncoder;
import com.team254.lib.util.MA3Encoder;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.SynchronousPID;

import edu.wpi.first.wpilibj.ContinuousRotationServo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hood extends Subsystem {
    private ContinuousRotationServo left_servo_;
    private ContinuousRotationServo right_servo_;
    private ContinuousRotationServo test_servo_;
    private MA3AnalogEncoder encoder_;
    private Solenoid stow_solenoid_;
    private boolean has_homed_;
    private SynchronousPID pid_;

    enum ControlMode {
        HOMING, OPEN_LOOP, POSITION
    }

    ControlMode control_mode_;

    Loop hood_loop_ = new Loop() {
        static final double kHomingTimeSeconds = 1.0;
        ControlMode last_iteration_control_mode_ = ControlMode.OPEN_LOOP;
        double homing_start_time_ = 0;

        @Override
        public void onLoop() {
            synchronized (Hood.this) {
                if (control_mode_ == ControlMode.HOMING) {
                    if (control_mode_ != last_iteration_control_mode_) {
                        startHoming();
                        homing_start_time_ = Timer.getFPGATimestamp();
                    } else if (Timer.getFPGATimestamp() >= homing_start_time_ + kHomingTimeSeconds) {
                        stopHoming(true);
                    }
                } else if (control_mode_ == ControlMode.POSITION) {
                    set(pid_.calculate(getAngle().getDegrees()));
                }
                last_iteration_control_mode_ = control_mode_;
            }
        }

        @Override
        public void onStart() {
            synchronized (Hood.this) {
                if (!has_homed_) {
                    control_mode_ = ControlMode.HOMING;
                }
            }
        }

        @Override
        public void onStop() {
            synchronized (Hood.this) {
                if (control_mode_ == ControlMode.HOMING) {
                    stopHoming(false);
                }
            }
        }
    };

    Hood() {
        left_servo_ = new ContinuousRotationServo(Constants.kOppositeSideServoPWM);
        right_servo_ = new ContinuousRotationServo(Constants.kSensorSideServoPWM);
        test_servo_ = new ContinuousRotationServo(Constants.kTestServoPWM);
        test_servo_.set(0.0);
        encoder_ = new MA3AnalogEncoder(Constants.kHoodEncoderAnalogId);
        pid_ = new SynchronousPID(Constants.kHoodKp, Constants.kHoodKi, Constants.kHoodKd);
        pid_.setDeadband(Constants.kHoodDeadband);
        pid_.setInputRange(Constants.kMinHoodAngle, Constants.kMaxHoodAngle);
        stow_solenoid_ = Constants.makeSolenoidForId(Constants.kHoodStowSolenoidId);

        has_homed_ = false;
        pid_.setSetpoint(Constants.kMinHoodAngle);
        control_mode_ = ControlMode.OPEN_LOOP;
    }

    Loop getLoop() {
        return hood_loop_;
    }

    synchronized void setDesiredAngle(Rotation2d angle) {
        if (control_mode_ != ControlMode.HOMING && control_mode_ != ControlMode.POSITION) {
            control_mode_ = ControlMode.POSITION;
            pid_.reset();
        }
        pid_.setSetpoint(angle.getDegrees());
    }

    public synchronized Rotation2d getAngle() {
        return Rotation2d.fromDegrees(
                encoder_.getContinuousAngleDegrees() * Constants.kHoodGearReduction + Constants.kMinHoodAngle);
    }

    private synchronized void set(double power) {
        left_servo_.set(-power);
        right_servo_.set(power);
    }

    synchronized void setOpenLoop(double power) {
        if (control_mode_ != ControlMode.HOMING) {
            set(power);
            control_mode_ = ControlMode.OPEN_LOOP;
        }
    }

    public synchronized void homeSystem() {
        control_mode_ = ControlMode.HOMING;
    }

    synchronized void startHoming() {
        control_mode_ = ControlMode.HOMING;
        set(-1.0);
    }

    synchronized void stopHoming(boolean success) {
        if (success) {
            has_homed_ = true;
            control_mode_ = ControlMode.POSITION;
            zeroSensors();
        } else {
            control_mode_ = ControlMode.OPEN_LOOP;
        }
        set(0);
    }

    public synchronized boolean hasHomed() {
        return has_homed_;
    }

    public synchronized boolean isOnTarget() {
        return (has_homed_ && control_mode_ == ControlMode.POSITION
                && Math.abs(pid_.getError()) < Constants.kHoodOnTargetTolerance);
    }

    public synchronized boolean isSafe() {
        return (control_mode_ == ControlMode.POSITION && getAngle().getDegrees() < Constants.kHoodMaxSafeAngle
                && pid_.getSetpoint() < Constants.kHoodMaxSafeAngle);
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putBoolean("has_hood_homed", has_homed_);
        SmartDashboard.putNumber("hood_angle", getAngle().getDegrees());
        SmartDashboard.putNumber("hood_setpoint", pid_.getSetpoint());
        SmartDashboard.putBoolean("hood_on_target", isOnTarget());
        SmartDashboard.putNumber("hood_error", pid_.getSetpoint() - getAngle().getDegrees());
    }

    @Override
    public synchronized void stop() {
        pid_.reset();
        control_mode_ = ControlMode.OPEN_LOOP;
        set(0);
    }

    @Override
    public synchronized void zeroSensors() {
        encoder_.zero();
    }

    public boolean isStowed() {
        return !stow_solenoid_.get();
    }

    void setStowed(boolean stow) {
        stow_solenoid_.set(!stow);
    }

    public void setTestServoSpeed(double speed) {
        test_servo_.set(speed);
    }
}
