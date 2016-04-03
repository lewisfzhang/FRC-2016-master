package com.team254.frc2016.subsystems;

import com.team254.frc2016.Constants;
import com.team254.frc2016.loops.Loop;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HoodRoller extends Subsystem {
    CANTalon mRollerTalon;
    AnalogInput mBallReadySensor;

    protected static enum WantedState {
        WANTS_STOP, WANTS_INTAKE, WANTS_REVERSE, WANTS_SHOOT
    };

    WantedState mWantedState = WantedState.WANTS_STOP;

    Loop mLoop = new Loop() {

        @Override
        public void onStart() {
            mWantedState = WantedState.WANTS_STOP;
        }

        @Override
        public void onLoop() {
            synchronized (HoodRoller.this) {
                switch (mWantedState) {
                case WANTS_INTAKE:
                    if (isBallPresent()) {
                        stop();
                    } else {
                        mRollerTalon.set(10.0);
                    }
                    break;
                case WANTS_REVERSE:
                    mRollerTalon.set(-12.0);
                    break;
                case WANTS_SHOOT:
                    mRollerTalon.set(10.0);
                    break;
                case WANTS_STOP:
                default:
                    stop();
                    break;
                }
            }
        }

        @Override
        public void onStop() {
        }

    };

    HoodRoller() {
        mRollerTalon = new CANTalon(Constants.kHoodRollerTalonId);
        mRollerTalon.enableBrakeMode(true);
        mRollerTalon.changeControlMode(CANTalon.TalonControlMode.Voltage);
        mBallReadySensor = new AnalogInput(Constants.kBallReadyAnalogId);
    }

    public Loop getLoop() {
        return mLoop;
    }

    public synchronized void intake() {
        mWantedState = WantedState.WANTS_INTAKE;
    }

    public synchronized void reverse() {
        mWantedState = WantedState.WANTS_REVERSE;
    }

    public synchronized void shoot() {
        mWantedState = WantedState.WANTS_SHOOT;
    }

    public boolean isBallPresent() {
        return mBallReadySensor.getAverageVoltage() > 2.0;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("ball_ready_sensor_voltage", mBallReadySensor.getAverageVoltage());
    }

    @Override
    public synchronized void stop() {
        mRollerTalon.set(0.0);
        mWantedState = WantedState.WANTS_STOP;
    }

    @Override
    public void zeroSensors() {
    }

}
