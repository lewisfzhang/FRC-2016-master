package edu.wpi.first.wpilibj;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

import edu.wpi.first.wpilibj.PWM;

// In the wpilibj package to use package-private methods from the base class.
// This is a driver for this servo:
// https://www.parallax.com/sites/default/files/downloads/900-00025-High-Speed-CR-Servo-Guide-v1.1.pdf
// Note that this requires modifying the servo so a 1.5ms pulse width is center (neutral)
public class ContinuousRotationServo extends PWM {
    InterpolatingTreeMap<Double, InterpolatingDouble> tree = new InterpolatingTreeMap<Double, InterpolatingDouble>();

    public ContinuousRotationServo(int channel) {
        super(channel);

        setBounds(1.3, 1.49, 1.5, 1.51, 1.7);

        // Static Values pulled from graph for 7.4V operation, then converted to
        // 6V
        // Key: RPM, Value: Pulse Width in ms
        tree.put(6 / 7.4 * 200.0, new InterpolatingDouble(1.3));
        tree.put(6 / 7.4 * 199.9, new InterpolatingDouble(1.325));
        tree.put(6 / 7.4 * 195.0, new InterpolatingDouble(1.35));
        tree.put(6 / 7.4 * 190.0, new InterpolatingDouble(1.375));
        tree.put(6 / 7.4 * 180.0, new InterpolatingDouble(1.4));
        tree.put(6 / 7.4 * 160.0, new InterpolatingDouble(1.425));
        tree.put(6 / 7.4 * 110.0, new InterpolatingDouble(1.45));
        tree.put(6 / 7.4 * 50.0, new InterpolatingDouble(1.475));
        tree.put(0.0, new InterpolatingDouble(1.5));
        tree.put(6 / 7.4 * -55.0, new InterpolatingDouble(1.525));
        tree.put(6 / 7.4 * -90.0, new InterpolatingDouble(1.55));
        tree.put(6 / 7.4 * -130.0, new InterpolatingDouble(1.575));
        tree.put(6 / 7.4 * -160.0, new InterpolatingDouble(1.6));
        tree.put(6 / 7.4 * -180.0, new InterpolatingDouble(1.625));
        tree.put(6 / 7.4 * -185.0, new InterpolatingDouble(1.65));
        tree.put(6 / 7.4 * -195.0, new InterpolatingDouble(1.675));
        tree.put(6 / 7.4 * -200.0, new InterpolatingDouble(1.7));
    }

    private double scaleOut(double pulsewidth) {
        // Takes in 1.3 thru 1.7 and outputs -1 to 1
        return (pulsewidth - 1.5) / (.4);
    }

    private double scaleIn(double input) {
        // Takes in -1 to 1 and scales it to 6/7.4 * (-200 to 200)
        return input * 200.0 * 6 / 7.4;
    }

    public void set(double value) {
        setSpeed(scaleOut(tree.getInterpolated(scaleIn(value)).value));
    }
}
