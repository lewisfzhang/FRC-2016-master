package com.team254.lib.util;

import edu.wpi.first.wpilibj.PWM;

public class ContinuousRotationServo extends PWM{
    InterpolatingTreeMap<Double, InterpolatingDouble> tree = new InterpolatingTreeMap<Double, InterpolatingDouble>();
    public ContinuousRotationServo(int channel) {
        super(channel);
        
        setBounds(1.3, 1.5, 1.5, 1.5, 1.7);
        
        //Static Values pulled from graph, Key: RPM, Value Pulse Width
        tree.put(200.0, new InterpolatingDouble(1.3));
        tree.put(199.9, new InterpolatingDouble(1.325));
        tree.put(195.0, new InterpolatingDouble(1.35));
        tree.put(190.0, new InterpolatingDouble(1.375));
        tree.put(180.0, new InterpolatingDouble(1.4));
        tree.put(160.0, new InterpolatingDouble(1.425));
        tree.put(110.0, new InterpolatingDouble(1.45));
        tree.put(50.0, new InterpolatingDouble(1.475));
        tree.put(0.0, new InterpolatingDouble(1.5));
        tree.put(-55.0, new InterpolatingDouble(1.525));
        tree.put(-90.0, new InterpolatingDouble(1.55));
        tree.put(-130.0, new InterpolatingDouble(1.575));
        tree.put(-160.0, new InterpolatingDouble(1.6));
        tree.put(-180.0, new InterpolatingDouble(1.625));
        tree.put(-185.0, new InterpolatingDouble(1.65));
        tree.put(-195.0, new InterpolatingDouble(1.675));
        tree.put(-200.0, new InterpolatingDouble(1.7));
    }
    
    private double scaleOut(Double pulsewidth) {
        //Takes in  1.3 thru 1.7 and outputs 0 to 1
        Double max = 1.7;
        Double min = 1.3;
        
        return (pulsewidth - min) * (1/(max-min));
    }
    
    private double scaleIn(Double input) {
        //Takes in -1 to 1 and scales it to -200 to 200
        return input*200.0;
    }
    
    public void set(double value) {
        //Between -1 and 1
        
        setPosition(scaleOut(tree.getInterpolated(scaleIn(value)).value));
        
        
    }
}
