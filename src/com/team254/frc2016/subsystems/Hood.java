package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;
import com.team254.frc2016.loops.Loop;
import com.team254.frc2016.loops.Looper;
import com.team254.lib.util.MA3Encoder;
import com.team254.lib.util.SynchronousPID;

import edu.wpi.first.wpilibj.ContinuousRotationServo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hood extends Subsystem {
    static Hood instance_ = new Hood();

    public static Hood getInstance() {
        return instance_;
    }

    ContinuousRotationServo left_servo_;
    ContinuousRotationServo right_servo_;
    MA3Encoder encoder_;
    boolean has_homed_;
    SynchronousPID pid_;

    enum ControlMode {
        HOMING, OPEN_LOOP, POSITION
    }

    ControlMode control_mode_;

    Loop hood_loop_ = new Loop() {
        static final long kHomingTimeNanos = 4000000000L; // 4 seconds
        ControlMode last_iteration_control_mode_ = ControlMode.OPEN_LOOP;
        long homing_start_time_ = 0;

        @Override
        public void onLoop() {
            synchronized (Hood.this) {
                if (control_mode_ == ControlMode.HOMING) {
                    if (control_mode_ != last_iteration_control_mode_) {
                        startHoming();
                        homing_start_time_ = System.nanoTime();
                    } else if (System.nanoTime() >= homing_start_time_ + kHomingTimeNanos) {
                        stopHoming(true);
                    }
                } else if (control_mode_ == ControlMode.POSITION) {
                    set(pid_.calculate(getAngle()));
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
        encoder_ = new MA3Encoder(Constants.kHoodEncoderDIO);
        pid_ = new SynchronousPID(Constants.kHoodKp, Constants.kHoodKi, Constants.kHoodKd);

        has_homed_ = false;
        pid_.setSetpoint(Constants.kMinHoodAngle);
        control_mode_ = ControlMode.OPEN_LOOP;

        Looper.getInstance().register(hood_loop_);
    }

    double getHoodGearRatio() {
        return 32 / 708; // TODO check with Colin
    }

    public synchronized void setDesiredAngle(double angle) {
        if (control_mode_ != ControlMode.HOMING && control_mode_ != ControlMode.POSITION) {
            control_mode_ = ControlMode.POSITION;
            pid_.reset();
        }
        pid_.setSetpoint(angle);
    }

    public synchronized double getAngle() {
        return encoder_.getContinuousAngleDegrees() * getHoodGearRatio() + Constants.kMinHoodAngle;
    }

    synchronized void set(double power) {
        left_servo_.set(power);
        right_servo_.set(-power);
    }

    public synchronized void setOpenLoop(double power) {
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
        set(1.0);
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

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putBoolean("has_hood_homed", has_homed_);
        SmartDashboard.putNumber("hood_angle", getAngle());
        SmartDashboard.putNumber("hood_setpoint", pid_.getSetpoint());
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
}
