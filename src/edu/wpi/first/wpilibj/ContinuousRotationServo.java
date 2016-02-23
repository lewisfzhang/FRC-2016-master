package edu.wpi.first.wpilibj;

import edu.wpi.first.wpilibj.PWM;

// In the wpilibj package to use package-private methods from the base class.
// This is a driver for a continuous rotation servo with neutral at 1500us pulse width.
public class ContinuousRotationServo extends PWM {
    public ContinuousRotationServo(int channel) {
        super(channel);

        setBounds(1.0, 1.48, 1.5, 1.52, 2.0);
        enableDeadbandElimination(true);
    }

    public void set(double value) {
        setSpeed(value);
    }
}
