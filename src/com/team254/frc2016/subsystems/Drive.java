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
        leftA = new CANTalon(Constants.kLeftDriveMasterId);
        leftB = new CANTalon(Constants.kLeftDriveSlaveId);
        rightA = new CANTalon(Constants.kRightDriveMasterId);
        rightB = new CANTalon(Constants.kRightDriveSlaveId);

        leftA.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        leftB.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        rightA.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        rightB.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        
        leftA.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        rightA.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);

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
    
    public int getLeftDistance() {
        return leftA.getEncPosition();
    }
    
    public int getRightDistance() {
        return rightA.getEncPosition();
    }
    
    public int getLeftVelocity() {
        return leftA.getEncVelocity();
    }
    
    public int getRightVelocity() {
        return rightA.getEncVelocity();
    }

    public void stop() {
        setLeftRightPower(0, 0);
    }
}
