package com.team254.lib.util;

import edu.wpi.first.wpilibj.*;

/**
 * A 12-bit PWM MA3 absolute encoder.
 * http://cdn.usdigital.com/assets/datasheets/MA3_datasheet.pdf
 * 
 * @author jarussell
 */
public class MA3AnalogEncoder {

    private final AnalogInput mAnalogInput;

    protected Notifier notifier_;
    protected Rotation2d rotation_ = new Rotation2d();
    protected Rotation2d home_ = new Rotation2d();
    protected int num_rotations_ = 0;

    private Runnable read_thread_ = new Runnable() {
        @Override
        public void run() {
            Rotation2d new_rotation = Rotation2d.fromRadians(2 * Math.PI * mAnalogInput.getVoltage() / 5.0);

            // Check for rollover
            synchronized (MA3AnalogEncoder.this) {
                double relative_angle = rotation_.getRadians()
                        + rotation_.inverse().rotateBy(new_rotation).getRadians();
                if (relative_angle > Math.PI) {
                    ++num_rotations_;
                    // System.out.println("Num rotations " + num_rotations_ + "
                    // angle " + new_rotation.getDegrees());
                } else if (relative_angle < -Math.PI) {
                    --num_rotations_;
                    // System.out.println("Num rotations " + num_rotations_ + "
                    // angle " + new_rotation.getDegrees());
                }
                rotation_ = new_rotation;
            }
        }
    };

    public MA3AnalogEncoder(int port) {
        mAnalogInput = new AnalogInput(port);
        notifier_ = new Notifier(read_thread_);
        notifier_.startPeriodic(0.01); // 100 Hz
    }

    public synchronized Rotation2d getCalibratedAngle() {
        return home_.rotateBy(rotation_);
    }

    public synchronized void zero() {
        num_rotations_ = 0;
        home_ = rotation_.inverse();
    }

    public synchronized Rotation2d getRawAngle() {
        return rotation_;
    }

    public synchronized double getContinuousAngleDegrees() {
        return getRawAngle().getDegrees() + num_rotations_ * 360.0 + home_.getDegrees();
    }

}