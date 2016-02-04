package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;
import edu.wpi.first.wpilibj.CANTalon;

public class Turret {
    private static Turret ourInstance = new Turret();

    public static Turret getInstance() {
        return ourInstance;
    }

    private CANTalon talon;

    private Turret() {
        talon = new CANTalon(Constants.kTurretTalonId);
        talon.changeControlMode(CANTalon.TalonControlMode.Position);
    }

    public void setAngleDegrees(double angle) {

    }
}
