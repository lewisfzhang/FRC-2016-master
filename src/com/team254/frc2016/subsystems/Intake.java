package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class Intake extends Subsystem {
    static Intake instance_ = new Intake();

    public static Intake getInstance() {
        return instance_;
    }

    private CANTalon intake_talon_;
    private CANTalon fixed_talon_;
    private Solenoid deploy_solenoid_;
    private boolean overridden_intake_ = false;

    private boolean deploy_safety_requirement_ = false;
    private double deploy_state_started_time_ = Timer.getFPGATimestamp();

    private Intake() {
        intake_talon_ = new CANTalon(Constants.kIntakeTalonId);
        intake_talon_.changeControlMode(TalonControlMode.PercentVbus);
        intake_talon_.enableBrakeMode(false);
        fixed_talon_ = new CANTalon(Constants.kFixedRollerTalonId);
        fixed_talon_.changeControlMode(TalonControlMode.PercentVbus);
        fixed_talon_.enableBrakeMode(false);
        deploy_solenoid_ = Constants.makeSolenoidForId(Constants.kIntakeSolenoidId);
    }

    synchronized void overrideIntaking(boolean overriding) {
        overridden_intake_ = overriding;
    }

    // Positive intakes balls, negative exhausts
    public synchronized void setIntakeRoller(double power) {
        if (!overridden_intake_) {
            intake_talon_.set(-power);
            fixed_talon_.set(-power);
        } else {
            intake_talon_.set(Math.max(-power, 0));
            fixed_talon_.set(Math.max(-power, 0));
        }
    }

    public synchronized void setDeploySafetyRequirement(boolean mustBeDeployed) {
        deploy_safety_requirement_ = mustBeDeployed;
        tryToDeploy(deploy_solenoid_.get());
    }

    public synchronized void tryToDeploy(boolean deploy) {
        deploy = deploy_safety_requirement_ || deploy;
        if (deploy == deploy_solenoid_.get()) {
            return;
        }
        deploy_solenoid_.set(deploy);
        deploy_state_started_time_ = Timer.getFPGATimestamp();
    }

    public synchronized boolean isDeployedAndSettled() {
        return deploy_solenoid_.get()
                && Timer.getFPGATimestamp() - deploy_state_started_time_
                > Constants.kIntakeDeploySettlingDelay;
    }

    @Override
    public synchronized void stop() {
        setIntakeRoller(0);
        tryToDeploy(false);
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
