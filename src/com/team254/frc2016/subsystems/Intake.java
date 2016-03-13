package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;

import com.team254.lib.util.ConstantsBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem {
    static Intake instance_ = new Intake();

    public static Intake getInstance() {
        return instance_;
    }

    private CANTalon intake_talon_;
    private CANTalon fixed_talon_;
    private Solenoid deploy_solenoid_;
    private boolean overridden_intake_ = false;
    private AnalogInput have_ball_sensor_;

    private Intake() {
        intake_talon_ = new CANTalon(Constants.kIntakeTalonId);
        intake_talon_.changeControlMode(TalonControlMode.PercentVbus);
        intake_talon_.enableBrakeMode(false);
        fixed_talon_ = new CANTalon(Constants.kFixedRollerTalonId);
        fixed_talon_.changeControlMode(TalonControlMode.PercentVbus);
        fixed_talon_.enableBrakeMode(false);
        deploy_solenoid_ = Constants.makeSolenoidForId(Constants.kIntakeSolenoidId);
        have_ball_sensor_ = new AnalogInput(Constants.kHaveBallSensorAnalogId);
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

    public synchronized void setDeploy(boolean deploy) {
        deploy_solenoid_.set(deploy);
    }

    @Override
    public synchronized void stop() {
        setIntakeRoller(0);
        setDeploy(false);
    }

    @Override
    public void outputToSmartDashboard() {
        double voltage = have_ball_sensor_.getAverageVoltage();
        // TODO: tune this threshold
        SmartDashboard.putBoolean("have_ball", voltage > 1.5);
        SmartDashboard.putNumber("have_ball_voltage", voltage);
    }

    @Override
    public void zeroSensors() {
        // no-op
    }
}
