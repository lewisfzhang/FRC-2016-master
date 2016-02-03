package com.team254.lib.util;

/**
 * Interpolable is an interface used by an Interpolating Tree to estimate linear
 * data based off of the two surrounding datapoints, by using the Interpolate
 * method.
 * 
 * @param <T>
 *            The Type of Interpolable
 * @see InterpolatingTreeMap
 */
public interface Interpolable<T> {
    /**
     * Interpolates between the surrounding two data points by taking the data
     * underneath (Low bound) the requested key and also the data above (High
     * Bound). Since keys are numbers in an interpolating tree, they can be used
     * to create an informed estimate. This method is always called on the
     * <strong>LOWER BOUND</strong>
     * 
     * Given the points (xValue, yValue) and (otherXValue, otherYValue), find
     * the point (interpolatedXValue, interpolatedYValue). - xValue,
     * otherXValue, otherYValue, and interpolatedXValue are given as parameters
     * - yValue should be known to this Interpolable - interpolatedYValue is the
     * return value of this function
     *
     * PRECONDITION: xValue <= interpolatedXValue <= otherXValue
     *
     * @param xValue
     *            The numerical key for the value that this Interpolable
     *            represents (Lower Bound)
     * @param otherXValue
     *            The numerical key for the value that the other Interpolable
     *            represents (Upper Bound)
     * @param otherYValue
     *            The value of the upper bound
     * @param interpolatedXValue
     *            The requested value.
     * @return Interpolable<T> The estimated average between the surrounding
     *         data
     */
    public T interpolate(Number xValue, Number otherXValue, T otherYValue, Number interpolatedXValue);
}