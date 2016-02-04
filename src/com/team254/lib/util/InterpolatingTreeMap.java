package com.team254.lib.util;

import java.util.TreeMap;
import java.util.Map;

/**
 * Interpolating Tree Maps are used to get values at points that are not defined
 * by making a guess from points that are defined
 * 
 * @param <K>
 *            The NUMERICAL type of the key (must extend number)
 * @param <T>
 *            The type of the value
 * @param <V>
 *            The INTERPOLABLE type of the value (must implement Interpolable)
 */
public class InterpolatingTreeMap<K extends Number, V extends Interpolable<V>> extends TreeMap<K, V> {
    private static final long serialVersionUID = 8347275262778054124L;

    Integer max_;

    public InterpolatingTreeMap(int maximumSize) {
        max_ = maximumSize;
    }

    public InterpolatingTreeMap() {
        this(0);
    }

    /**
     * Inserts a key value pair, and trims the tree if a max size is specified
     * 
     * @param key
     *            Key for inserted data
     * @param value
     *            Value for inserted data
     * @return the value
     */
    @Override
    public V put(K key, V value) {
        if (max_ != null && max_ <= size()) {
            // "Prune" the tree if it is oversize
            K first = firstKey();
            remove(first);
        }

        super.put(key, value);

        return value;
    }

    @Override
    public void putAll(Map<? extends K, ? extends V> map) {
        System.out.println("Unimplemented Method");
    }

    /**
     *
     * @param key
     *            Lookup for a value (does not have to exist)
     * @return V or null; V if it is Interpolable or exists, null if it is at a
     *         bound and cannot average
     */
    public V getInterpolated(K key) {
        V gotval = get(key);
        if (gotval == null) {
            // Get surrounding keys for interpolation
            K topBound = ceilingKey(key);
            K bottomBound = floorKey(key);

            // If attempting interpolation at ends of tree, return the nearest
            // data point
            if (topBound == null && bottomBound == null) {
                return null;
            } else if (topBound == null) {
                return get(bottomBound);
            } else if (bottomBound == null) {
                return get(topBound);
            }

            // Get surrounding values for interpolation
            V topElem = get(topBound);
            V bottomElem = get(bottomBound);

            // Scale the key to the interval [0, 1]
            double scaledKey = 0;

            // The Number class (and Java) leaves a lot to be desired...
            if (key instanceof Double || key instanceof Float) {
                scaledKey = (key.doubleValue() - bottomBound.doubleValue())
                        / (topBound.doubleValue() - bottomBound.doubleValue());
            } else {
                // Assume an integer type
                scaledKey = (key.longValue() - bottomBound.longValue())
                        / ((double) (topBound.longValue() - bottomBound.longValue()));
            }
            return (V) bottomElem.interpolate(topElem, scaledKey);
        } else {
            return gotval;
        }
    }
}