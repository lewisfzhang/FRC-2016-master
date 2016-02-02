package com.team254.lib.util;

import java.util.TreeMap;
import java.util.Map;
import java.lang.Exception;

/**
 * Interpolating Tree Maps are used to get values at points that are not defined by making a guess from points that are defined
 * @param <K> The NUMERICAL type of the key (must extend number)
 * @param <V> The AVERAGEABLE type of the value (must implement Averageable)
 */
public class InterpolatingTreeMap<K extends Number, V extends Averageable> extends TreeMap<K, V> {
    Integer max;
    
    public InterpolatingTreeMap(int maximumSize) {
        max = maximumSize;
    }
    
    public InterpolatingTreeMap() {}

    /**
     * Inserts a key value pair, and trims the tree if a max size is specified
     * @param key Key for inserted data
     * @param value Value for inserted data
     * @return the value
     */
    public V put(K key, V value) {
        if (max != null && max <= size()) {
            //"Prune" the tree if it is oversize
            K first = firstKey();
            remove(first);
        }
        
        super.put(key, value);
        
        return value;
    }
    
    public void putAll(Map<? extends K,? extends V> map) {
        System.out.println("Unimplemented Method");
    }

    /**
     *
     * @param key Lookup for a value (does not have to exist)
     * @return V or null; V if it is averageable or exists, null if it is at a bound and cannot average
     */
    public V getInterpolated(K key) {
        V gotval = get(key);
        if (gotval == null) {
            //Get surrounding keys for interpolation
            K topBound = ceilingKey(key);
            K bottomBound = floorKey(key);
            
            //If attempting interploation at ends of tree, return null
            if (topBound == null || bottomBound == null)
                return null;
                
            //Get surrounding values for interpolation
            V topElem = get(topBound);
            V bottomElem = get(bottomBound);
            
            //Number xValue, Number highXValue, Number highYValue, Number searchedForValue
            return (V) bottomElem.average((Number) bottomBound, (Number) topBound, topElem, (Number) key);
        } else {
            return gotval;
        }
    }
}