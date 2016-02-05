package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;
import edu.wpi.first.wpilibj.CANTalon;

public class Turret {
    private static Turret instance_ = new Turret();

    public static Turret getInstance() {
        return instance_;
    }

    private CANTalon talon;

    private Turret() {
        talon = new CANTalon(Constants.kTurretTalonId);
        talon.changeControlMode(CANTalon.TalonControlMode.Position);
    }

    public void setAngleDegrees(double angle) {

    }
}
