package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.CANTalon;

public class Drive {

    private static Drive ourInstance = new Drive();

    public static Drive getInstance() {
        return ourInstance;
    }

    private final CANTalon leftA, leftB, rightA, rightB;

    private Drive() {
        leftA = new CANTalon(Constants.kLeftDriveAId);
        leftB = new CANTalon(Constants.kLeftDriveBId);
        rightA = new CANTalon(Constants.kRightDriveAId);
        rightB = new CANTalon(Constants.kRightDriveBId);

        leftA.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        leftB.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        rightA.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        rightB.changeControlMode(CANTalon.TalonControlMode.PercentVbus);

        leftA.enableBrakeMode(false);
        leftB.enableBrakeMode(false);
        rightA.enableBrakeMode(false);
        rightB.enableBrakeMode(false);
    }

    public void setLeftRightPower(double left, double right) {
        leftA.set(left);
        leftB.set(left);
        rightA.set(-right);
        rightB.set(-right);
    }

    public void set(DriveSignal signal) {
        setLeftRightPower(signal.leftMotor, signal.rightMotor);
    }

    public void stop() {
        setLeftRightPower(0, 0);
    }
}
