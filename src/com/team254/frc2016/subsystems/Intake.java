package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;

public class Intake {
    static Intake instance_ = new Intake();

    public static Intake getInstance() {
        return instance_;
    }
    
    CANTalon intake_talon_;
    
    Intake() {
        intake_talon_ = new CANTalon(Constants.kIntakeTalonId);
        intake_talon_.changeControlMode(TalonControlMode.PercentVbus);
    }
    
    public synchronized void set(double power) {
        intake_talon_.set(power);
    }
    
    public synchronized void stop() {
        set(0);
    }
}
