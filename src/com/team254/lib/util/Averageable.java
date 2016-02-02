package com.team254.lib.util;

/**
 * Averageable is an interface used by an Interpolating Tree to estimate linear
 * data based off of the two surrounding datapoints, By using the average method
 * 
 * @param <T>
 *            The Type of averageable
 * @see InterpolatingTreeMap
 */
public interface Averageable<T> {

    /**
     * Averages the surrounding two data by taking the data underneath (Low
     * bound) the requested key and also the data above (High Bound). Since keys
     * are numbers in an interpolating tree, they can be used to create an
     * informed estimate This method is always called on the <strong>LOWER
     * BOUND</strong>
     *
     * @param xValue
     *            The numerical key for the value that this averageable
     *            represents (Lower Bound)
     * @param highXValue
     *            The numerical key for the value that this highYValue
     *            represents (Upper Bound)
     * @param highYValue
     *            The value of the upper bound
     * @param searchedForValue
     *            The requested value
     * @return Averageable<T> The estimated average between the surrounding data
     */
    public Averageable<T> average(Number xValue, Number highXValue, Averageable<T> highYValue, Number searchedForValue);
}