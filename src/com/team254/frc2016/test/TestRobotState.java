package com.team254.frc2016.test;

import static org.junit.Assert.*;

import org.junit.Test;

import com.team254.frc2016.Constants;
import com.team254.frc2016.RobotState;
import com.team254.lib.util.Pose2d;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

public class TestRobotState {
    public static final double kTestEpsilon = 1E-9;

    @Test
    public void smokeTest() {
        // Start at the identity.
        long start_time = System.nanoTime();
        RobotState rs = new RobotState(start_time, new Pose2d(), new Rotation2d());
        System.out.println("Latest time is " + rs.getLatestOdometricToVehicle().getKey());

        // Check the latest state.
        assert (rs.getLatestOdometricToVehicle().getKey() == start_time);
        assertEquals(0, rs.getLatestOdometricToVehicle().getValue().getTranslation().getX(), kTestEpsilon);
        assertEquals(0, rs.getLatestOdometricToVehicle().getValue().getTranslation().getY(), kTestEpsilon);
        assertEquals(0, rs.getLatestOdometricToVehicle().getValue().getRotation().getRadians(), kTestEpsilon);
        assert (rs.getLatestTurretRotation().getKey() == start_time);
        assertEquals(0, rs.getLatestTurretRotation().getValue().getRadians(), kTestEpsilon);

        // Add a new measurement 10 milliseconds later
        long next_time = rs.getLatestOdometricToVehicle().getKey() + 10000000;
        System.out.println("Next time is " + next_time);
        rs.addObservations(next_time, new Pose2d(new Translation2d(1, 0), Rotation2d.fromDegrees(10)),
                Rotation2d.fromDegrees(-15));

        // Check the latest state.
        assertEquals(next_time, rs.getLatestOdometricToVehicle().getKey().longValue());
        assertEquals(1, rs.getLatestOdometricToVehicle().getValue().getTranslation().getX(), kTestEpsilon);
        assertEquals(0, rs.getLatestOdometricToVehicle().getValue().getTranslation().getY(), kTestEpsilon);
        assertEquals(10, rs.getLatestOdometricToVehicle().getValue().getRotation().getDegrees(), kTestEpsilon);
        assertEquals(1, rs.getOdometricToVehicle(next_time).getTranslation().getX(), kTestEpsilon);
        assertEquals(0, rs.getOdometricToVehicle(next_time).getTranslation().getY(), kTestEpsilon);
        assertEquals(10, rs.getOdometricToVehicle(next_time).getRotation().getDegrees(), kTestEpsilon);
        assertEquals(next_time, rs.getLatestTurretRotation().getKey().longValue());
        assertEquals(-15, rs.getLatestTurretRotation().getValue().getDegrees(), kTestEpsilon);
        assertEquals(-15, rs.getTurretRotation(next_time).getDegrees(), kTestEpsilon);

        // Test interpolation
        long interp_time = next_time - 5000000;
        System.out.println("Interp time is " + interp_time);
        assertEquals(0.5, rs.getOdometricToVehicle(interp_time).getTranslation().getX(), kTestEpsilon);
        assertEquals(0, rs.getOdometricToVehicle(interp_time).getTranslation().getY(), kTestEpsilon);
        assertEquals(5, rs.getOdometricToVehicle(interp_time).getRotation().getDegrees(), kTestEpsilon);
        assertEquals(-7.5, rs.getTurretRotation(interp_time).getDegrees(), kTestEpsilon);
    }

    @Test
    public void simulatedTest() {
        long start_time = System.nanoTime();
        RobotState rs = new RobotState(start_time, new Pose2d(), new Rotation2d());

        final long kTimestep = 10000000; // 10ms

        // Travel 360 units in distance while rotating 360 degrees CW (ex.
        // driving an arc to the right)
        // Meanwhile, the turret rotates 360 degrees CCW.
        for (int i = 1; i <= 360; ++i) {
            long timestamp = start_time + i * kTimestep;
            rs.addObservations(timestamp, rs.generateOdometryFromSensors(1, 1, Rotation2d.fromDegrees(-i)),
                    Rotation2d.fromDegrees(i));
            System.out.println("t = " + i * 10 + " ms");
            System.out.println("Odometric to vehicle: " + rs.getOdometricToVehicle(timestamp));
            System.out.println("Odometric to turret rotated: " + rs.getOdometricToTurretRotated(timestamp));
            System.out.println("Odometric to camera: " + rs.getOdometricToCamera(timestamp));
            System.out.println("Turret rotation: " + rs.getTurretRotation(timestamp));

            // The turret is constantly counter-acting vehicle rotation. Check
            // that the camera pose makes sense.
            assertEquals(0, rs.getOdometricToCamera(timestamp).getRotation().getDegrees(), kTestEpsilon);
            assertEquals(Constants.kCameraXOffset, rs.getOdometricToCamera(timestamp).getTranslation().getX()
                    - rs.getOdometricToTurretRotated(timestamp).getTranslation().getX(), kTestEpsilon);
        }
        // We should end up where we started
        assertEquals(0, rs.getLatestOdometricToVehicle().getValue().getTranslation().getX(), kTestEpsilon);
        assertEquals(0, rs.getLatestOdometricToVehicle().getValue().getTranslation().getY(), kTestEpsilon);
        assertEquals(0, rs.getLatestOdometricToVehicle().getValue().getRotation().getDegrees(), kTestEpsilon);
        assertEquals(0, rs.getLatestTurretRotation().getValue().getDegrees(), kTestEpsilon);
    }
}
