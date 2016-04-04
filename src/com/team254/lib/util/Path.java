package com.team254.lib.util;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

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
        if (mSegments.size() == 0) {
            return new Translation2d();
        }

        // Check the distances to the start and end of each segment. As soon as
        // we find a point > lookahead_distance away, we know the right point
        // lies somewhere on that segment.
        Translation2d position_inverse = position.inverse();
        if (position_inverse.translateBy(mSegments.get(0).getStart()).norm() >= lookahead_distance) {
            // Special case: Before the first point, so just return the first
            // point.
            return mSegments.get(0).getStart();
        }
        for (int i = 0; i < mSegments.size(); ++i) {
            PathSegment segment = mSegments.get(i);
            double distance = position_inverse.translateBy(segment.getEnd()).norm();
            if (distance >= lookahead_distance) {
                // This segment contains the lookahead point
                Optional<Translation2d> intersection_point = getFirstCircleSegmentIntersection(segment, position,
                        lookahead_distance);
                if (intersection_point.isPresent()) {
                    return intersection_point.get();
                } else {
                    System.out.println("ERROR: No intersection point?");
                }
            }
        }
        // Special case: After the last point, so extrapolate forward.
        PathSegment last_segment = mSegments.get(mSegments.size() - 1);
        PathSegment new_last_segment = new PathSegment(last_segment.getStart(), last_segment.interpolate(10000));
        Optional<Translation2d> intersection_point = getFirstCircleSegmentIntersection(new_last_segment, position,
                lookahead_distance);
        if (intersection_point.isPresent()) {
            return intersection_point.get();
        } else {
            System.out.println("ERROR: No intersection point anywhere on line?");
            return last_segment.getEnd();
        }
    }

    static Optional<Translation2d> getFirstCircleSegmentIntersection(PathSegment segment, Translation2d center,
            double radius) {
        double x1 = segment.getStart().getX() - center.getX();
        double y1 = segment.getStart().getY() - center.getY();
        double x2 = segment.getEnd().getX() - center.getX();
        double y2 = segment.getEnd().getY() - center.getY();
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dr_squared = dx * dx + dy * dy;
        double det = x1 * y2 - x2 * y1;

        double discriminant = dr_squared * radius * radius - det * det;
        if (discriminant < 0) {
            // No intersection
            return Optional.empty();
        }

        double sqrt_discriminant = Math.sqrt(discriminant);
        Translation2d pos_solution = new Translation2d(
                (det * dy + (dy < 0 ? -1 : 1) * dx * sqrt_discriminant) / dr_squared + center.getX(),
                (-det * dx + Math.abs(dy) * sqrt_discriminant) / dr_squared + center.getY());
        Translation2d neg_solution = new Translation2d(
                (det * dy - (dy < 0 ? -1 : 1) * dx * sqrt_discriminant) / dr_squared + center.getX(),
                (-det * dx - Math.abs(dy) * sqrt_discriminant) / dr_squared + center.getY());

        System.out.println("Pos solution " + pos_solution + ", neg solution " + neg_solution);

        // Choose the one between start and end that is closest to start
        double pos_dot_product = segment.dotProduct(pos_solution);
        double neg_dot_product = segment.dotProduct(neg_solution);
        if (pos_dot_product < 0 && neg_dot_product >= 0) {
            return Optional.of(neg_solution);
        } else if (pos_dot_product >= 0 && neg_dot_product < 0) {
            return Optional.of(pos_solution);
        } else {
            if (Math.abs(pos_dot_product) <= Math.abs(neg_dot_product)) {
                return Optional.of(pos_solution);
            } else {
                return Optional.of(neg_solution);
            }
        }
    }
}
