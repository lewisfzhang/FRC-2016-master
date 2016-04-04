package com.team254.lib.util;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class Path {
    protected static final double kSegmentCompletePercentage = .99;

    protected List<PathSegment> mSegments;

    public Path(List<Translation2d> waypoints) {
        mSegments = new ArrayList<PathSegment>();
        for (int i = 0; i < waypoints.size() - 1; ++i) {
            mSegments.add(new PathSegment(waypoints.get(i), waypoints.get(i + 1)));
        }
    }

    // Returns the distance from the position to the first point on the path
    public double update(Translation2d position) {
        double rv = 0.0;
        for (Iterator<PathSegment> it = mSegments.iterator(); it.hasNext();) {
            PathSegment segment = it.next();
            PathSegment.ClosestPointReport closest_point_report = segment.getClosestPoint(position);
            if (closest_point_report.index >= kSegmentCompletePercentage) {
                // This segment is complete and can be removed.
                it.remove();
            } else {
                if (closest_point_report.index > 0.0) {
                    // Can shorten this segment
                    segment.updateStart(closest_point_report.closest_point);
                }
                // We are done
                rv = closest_point_report.distance;
                break;
            }
        }
        return rv;
    }

    public double getRemainingLength() {
        double length = 0.0;
        for (int i = 0; i < mSegments.size(); ++i) {
            length += mSegments.get(i).getLength();
        }
        return length;
    }

    public Translation2d getLookaheadPoint(Translation2d position, double lookahead_distance) {
        Translation2d rv = new Translation2d();
        if (mSegments.size() == 0) {
            return rv;
        }

        // Check the distances to the start and end of each segment. As soon as
        // we find a point > lookahead_distance away, we know the right point
        // lies somewhere on that segment.
        Translation2d position_inverse = position.inverse();
        if (position_inverse.translateBy(mSegments.get(0).getStart()).norm() >= lookahead_distance) {
            // Special case: Before the first point
            // TODO
        } else {
            boolean found_segment = false;
            for (int i = 0; i < mSegments.size(); ++i) {
                PathSegment segment = mSegments.get(i);
                double distance = position_inverse.translateBy(segment.getEnd()).norm();
                if (distance >= lookahead_distance) {
                    // This segment contains the lookahead point
                    // TODO
                    found_segment = true;
                }
            }
            if (!found_segment) {
                // Special case: After the last point
                // TODO
            }
        }

        return rv;
    }

    static double getFirstCircleSegmentIntersection(PathSegment segment, Translation2d center, double radius) {
        // TODO
        return 0.0;
    }
}
