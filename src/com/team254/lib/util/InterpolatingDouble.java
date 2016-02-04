package com.team254.lib.util;

public class InterpolatingDouble implements Interpolable<InterpolatingDouble> {
    public Double value = 0.0;

    public InterpolatingDouble(Double val) {
        value = val;
    }

    @Override
    public InterpolatingDouble interpolate(Number xValue, Number otherXValue, InterpolatingDouble otherYValue,
            Number interpolatedXValue) {
        Double dy = otherYValue.value - value;
        Double dx = otherXValue.doubleValue() - xValue.doubleValue();

        Double searchY = dy / dx * (interpolatedXValue.doubleValue() - xValue.doubleValue()) + value;

        return new InterpolatingDouble(searchY);
    }

}