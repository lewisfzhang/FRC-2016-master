package com.team254.lib.util.test;

import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.util.*;

import org.junit.Test;

public class PathTest {
    public static final double kTestEpsilon = 1E-9;

    @Test
    public void testPathSegment() {
        Translation2d start = new Translation2d(0, 0);
        Translation2d end = new Translation2d(1, 0);
        PathSegment segment = new PathSegment(start, end);
        assertEquals(1, segment.getLength(), kTestEpsilon);
        assertEquals(start.getX(), segment.getStart().getX(), kTestEpsilon);
        assertEquals(start.getY(), segment.getStart().getY(), kTestEpsilon);
        assertEquals(end.getX(), segment.getEnd().getX(), kTestEpsilon);
        assertEquals(end.getY(), segment.getEnd().getY(), kTestEpsilon);

        // Update start
        start = new Translation2d(0.5, 0);
        segment.updateStart(start);
        assertEquals(0.5, segment.getLength(), kTestEpsilon);
        assertEquals(start.getX(), segment.getStart().getX(), kTestEpsilon);
        assertEquals(start.getY(), segment.getStart().getY(), kTestEpsilon);

        // Interpolate
        Translation2d midpoint = segment.interpolate(0.5);
        assertEquals(.75, midpoint.getX(), kTestEpsilon);
        assertEquals(0, midpoint.getY(), kTestEpsilon);

        // GetClosestPoint - point on path
        PathSegment.ClosestPointReport report = segment.getClosestPoint(midpoint);
        assertEquals(.5, report.index, kTestEpsilon);
        assertEquals(.5, report.clamped_index, kTestEpsilon);
        assertEquals(midpoint.getX(), report.closest_point.getX(), kTestEpsilon);
        assertEquals(midpoint.getY(), report.closest_point.getY(), kTestEpsilon);
        assertEquals(0, report.distance, kTestEpsilon);

        // GetClosestPoint - point off of path
        report = segment.getClosestPoint(new Translation2d(.75, 1));
        assertEquals(.5, report.index, kTestEpsilon);
        assertEquals(.5, report.clamped_index, kTestEpsilon);
        assertEquals(midpoint.getX(), report.closest_point.getX(), kTestEpsilon);
        assertEquals(midpoint.getY(), report.closest_point.getY(), kTestEpsilon);
        assertEquals(1, report.distance, kTestEpsilon);

        // GetClosestPoint - point behind start
        report = segment.getClosestPoint(new Translation2d(0, 1));
        assertEquals(-1, report.index, kTestEpsilon);
        assertEquals(0, report.clamped_index, kTestEpsilon);
        assertEquals(start.getX(), report.closest_point.getX(), kTestEpsilon);
        assertEquals(start.getY(), report.closest_point.getY(), kTestEpsilon);
        assertEquals(Math.hypot(.5, 1), report.distance, kTestEpsilon);

        // GetClosestPoint - point after end
        report = segment.getClosestPoint(new Translation2d(2, -1));
        assertEquals(3, report.index, kTestEpsilon);
        assertEquals(1, report.clamped_index, kTestEpsilon);
        assertEquals(end.getX(), report.closest_point.getX(), kTestEpsilon);
        assertEquals(end.getY(), report.closest_point.getY(), kTestEpsilon);
        assertEquals(Math.hypot(1, 1), report.distance, kTestEpsilon);
    }

    @Test
    public void testPath() {
        List<Translation2d> waypoints = new ArrayList<>();
        waypoints.add(new Translation2d(0, 0));
        waypoints.add(new Translation2d(1, 0));
        waypoints.add(new Translation2d(2, 0));
        waypoints.add(new Translation2d(2, 1));
        waypoints.add(new Translation2d(2, 2));

        Path path = new Path(waypoints);
        assertEquals(4, path.getRemainingLength(), kTestEpsilon);

        Translation2d robot_position = new Translation2d(0, 0);
        path.update(robot_position);
        assertEquals(4, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Translation2d(.5, 0);
        path.update(robot_position);
        assertEquals(3.5, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Translation2d(1, 0);
        path.update(robot_position);
        assertEquals(3, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Translation2d(1, .5);
        path.update(robot_position);
        assertEquals(3, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Translation2d(2.5, .5);
        path.update(robot_position);
        assertEquals(1.5, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Translation2d(0, 0);
        path.update(robot_position);
        assertEquals(1.5, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Translation2d(2.5, 2.5);
        path.update(robot_position);
        assertEquals(0, path.getRemainingLength(), kTestEpsilon);

        robot_position = new Translation2d(0, 0);
        path.update(robot_position);
        assertEquals(0, path.getRemainingLength(), kTestEpsilon);
    }

    @Test
    public void testLookahead() {
        List<Translation2d> waypoints = new ArrayList<>();
        waypoints.add(new Translation2d(0, 0));
        waypoints.add(new Translation2d(1, 0));
        waypoints.add(new Translation2d(2, 0));
        waypoints.add(new Translation2d(2, 1));
        waypoints.add(new Translation2d(2, 2));
        Path path = new Path(waypoints);

        // Robot at path start, lookahead 1 unit
        Translation2d robot_position = new Translation2d(0, 0);
        path.update(robot_position);
        Translation2d lookahead_point = path.getLookaheadPoint(robot_position, 1);
        assertEquals(1, lookahead_point.getX(), kTestEpsilon);
        assertEquals(0, lookahead_point.getY(), kTestEpsilon);

        // Robot at path start, lookahead 2 units
        robot_position = new Translation2d(0, 0);
        path.update(robot_position);
        lookahead_point = path.getLookaheadPoint(robot_position, 2);
        assertEquals(2, lookahead_point.getX(), kTestEpsilon);
        assertEquals(0, lookahead_point.getY(), kTestEpsilon);

        // Robot at path start, lookahead 2.1 units
        robot_position = new Translation2d(0, 0);
        path.update(robot_position);
        lookahead_point = path.getLookaheadPoint(robot_position, 2.1);
        assertEquals(2, lookahead_point.getX(), kTestEpsilon);
        assertTrue(0 < lookahead_point.getY());

        // Robot near path start, lookahead 1 unit
        robot_position = new Translation2d(0, 0.1);
        path.update(robot_position);
        lookahead_point = path.getLookaheadPoint(robot_position, 1);
        assertTrue(1 > lookahead_point.getX());
        assertEquals(0, lookahead_point.getY(), kTestEpsilon);

        // Robot behind path start, lookahead 1 unit
        robot_position = new Translation2d(-.5, 0);
        path.update(robot_position);
        lookahead_point = path.getLookaheadPoint(robot_position, 1);
        assertEquals(.5, lookahead_point.getX(), kTestEpsilon);
        assertEquals(0, lookahead_point.getY(), kTestEpsilon);

        // Lookahead goes past end
        robot_position = new Translation2d(0, 0);
        path.update(robot_position);
        lookahead_point = path.getLookaheadPoint(robot_position, 5);
        assertEquals(2, lookahead_point.getX(), kTestEpsilon);
        assertTrue(2 < lookahead_point.getY());
    }
}
