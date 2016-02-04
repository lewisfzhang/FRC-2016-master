package com.team254.lib.util;

public class InterpolatingDouble implements Interpolable<InterpolatingDouble> {
    public Double value = 0.0;

    public InterpolatingDouble(Double val) {
        value = val;
    }

    @Override
    public InterpolatingDouble interpolate(InterpolatingDouble other, double x) {
        Double dydx = other.value - value;
        Double searchY = dydx * x + value;
        return new InterpolatingDouble(searchY);
    }

}