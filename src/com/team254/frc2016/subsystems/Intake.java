package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake extends Subsystem {
    static Intake instance_ = new Intake();

    public static Intake getInstance() {
        return instance_;
    }

    CANTalon intake_talon_;
    CANTalon fixed_talon_;
    Solenoid solenoid_;
    boolean overridden_ = false;

    Intake() {
        intake_talon_ = new CANTalon(Constants.kIntakeTalonId);
        intake_talon_.changeControlMode(TalonControlMode.PercentVbus);
        intake_talon_.enableBrakeMode(false);
        fixed_talon_ = new CANTalon(Constants.kFixedRollerTalonId);
        fixed_talon_.changeControlMode(TalonControlMode.PercentVbus);
        fixed_talon_.enableBrakeMode(false);
        solenoid_ = new Solenoid(Constants.kIntakeSolenoidId / 8, Constants.kIntakeSolenoidId % 8);
    }

    synchronized void overrideIntaking(boolean enable) {
        overridden_ = enable;
    }

    // Positive intakes balls, negative exhausts
    public synchronized void set(double power) {
        if (!overridden_) {
            intake_talon_.set(-power);
            fixed_talon_.set(-power);
        } else {
            intake_talon_.set(0);
            fixed_talon_.set(0);
        }
    }

    public void deploy(boolean deploy) {
        solenoid_.set(deploy);
    }

    @Override
    public synchronized void stop() {
        set(0);
        deploy(false);
    }

    @Override
    public void outputToSmartDashboard() {
        // no-op
    }

    @Override
    public void zeroSensors() {
        // no-op
    }
}
