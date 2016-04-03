package com.team254.lib.util;

public class PathSegment {
    protected static final double kEpsilon = 1E-9;

    protected Translation2d mStart;
    protected Translation2d mEnd;
    protected Translation2d mStartToEnd; // pre-computed for efficiency
    protected double mLength; // pre-computed for efficiency

    public static class ClosestPointReport {
        public double index; // Index of the point on the path segment (not
                             // clamped to [0, 1])
        public double clamped_index; // As above, but clamped to [0, 1]
        public Translation2d closest_point; // The result of
                                            // interpolate(clamped_index)
        public double distance; // The distance from closest_point to the query
                                // point
    }

    public PathSegment(Translation2d start, Translation2d end) {
        mEnd = end;
        updateStart(start);
    }

    public void updateStart(Translation2d new_start) {
        mStart = new_start;
        mStartToEnd = mStart.inverse().translateBy(mEnd);
        mLength = mStartToEnd.norm();
    }

    public Translation2d getStart() {
        return mStart;
    }

    public Translation2d getEnd() {
        return mEnd;
    }

    public double getLength() {
        return mLength;
    }

    // Index is on [0, 1]
    public Translation2d interpolate(double index) {
        return mStart.interpolate(mEnd, index);
    }

    public ClosestPointReport getClosestPoint(Translation2d query_point) {
        ClosestPointReport rv = new ClosestPointReport();
        if (mLength > kEpsilon) {
            Translation2d start_to_query = mStart.inverse().translateBy(query_point);
            double dot_product = mStartToEnd.getX() * start_to_query.getX() + mStartToEnd.getY() * start_to_query.getY();
            rv.index = dot_product / mLength;
            rv.clamped_index = Math.min(1.0, Math.max(0.0, rv.index));
            rv.closest_point = interpolate(rv.index);
        } else {
            rv.index = rv.clamped_index = 0.0;
            rv.closest_point = new Translation2d(mStart);
        }
        rv.distance = rv.closest_point.inverse().translateBy(query_point).norm();
        return rv;
    }
}