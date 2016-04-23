package com.team254.frc2016.auto.modes;

import com.team254.frc2016.RobotState;
import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.*;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Superstructure;
import com.team254.frc2016.subsystems.UtilityArm;
import com.team254.lib.util.Path;
import com.team254.lib.util.RigidTransform2d;
import com.team254.lib.util.Translation2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This mode goes over the rock wall and tries to 2 ball
 */
public class TwoBallRockWallMode extends AutoModeBase {

    Drive mDrive = Drive.getInstance();
    Superstructure mSuperstructure = Superstructure.getInstance();

    public static final double DISTANCE_TO_DROP_ARM = 70;
    public static final double DISTANCE_TO_SLOW_OVER_BUMP = 95;
    public static final double DISTANCE_TO_SPEED_UP_AGAIN = 110;
    public double mDistanceToDrive = 162;

    @Override
    protected void routine() throws AutoModeEndedException {
        // Make paths
        List<Path.Waypoint> go_to_line_path = new ArrayList<>();
        go_to_line_path.add(new Path.Waypoint(new Translation2d(0, 0), 25.0));
        go_to_line_path.add(new Path.Waypoint(new Translation2d(-50, 0), 25.0));

        List<Path.Waypoint> first_path = new ArrayList<>();
        first_path.add(new Path.Waypoint(new Translation2d(0, 0), 84.0));
        first_path.add(new Path.Waypoint(new Translation2d(45, 0), 52.0));
        first_path.add(new Path.Waypoint(new Translation2d(DISTANCE_TO_DROP_ARM, 0), 52.0, "DropArm"));
        first_path.add(new Path.Waypoint(new Translation2d(DISTANCE_TO_SLOW_OVER_BUMP, 0), 36.0));
        first_path.add(new Path.Waypoint(new Translation2d(DISTANCE_TO_SPEED_UP_AGAIN, 0), 80.0));
        first_path.add(new Path.Waypoint(new Translation2d(mDistanceToDrive, 0), 80.0));

        List<Path.Waypoint> return_path = new ArrayList<>();
        return_path.add(new Path.Waypoint(new Translation2d(mDistanceToDrive, 0), 55.0));
        return_path.add(new Path.Waypoint(new Translation2d(15, 0), 54.0, "WatchLine"));
        return_path.add(new Path.Waypoint(new Translation2d(6, 0), 25.0));
        return_path.add(new Path.Waypoint(new Translation2d(-100, 0), 25.0));

        // Start robot actions
        runAction(
                new ParallelAction(
                        Arrays.asList(new FollowPathAction(new Path(first_path), false),
                                new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("DropArm"),
                                        new SetArmModeAction(UtilityArm.WantedState.LOW_BAR),
                                        new StartAutoAimingAction())))));

        // Shoot ball
        runAction(new WaitAction(.5));
        runAction(new ShootWhenReadyAction());
        mSuperstructure.setWantedState(Superstructure.WantedState.WANT_TO_KEEP_SPINNING);
        runAction(new SetArmModeAction(UtilityArm.WantedState.DRIVING));

        // Drive back to center line
        runAction(new ParallelAction(Arrays.asList(new FollowPathAction(new Path(return_path), true),
                new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("WatchLine"), new WaitUntilLineAction())))));

        // Creep off the line
        RigidTransform2d lineRobotPose = RobotState.getInstance().getLatestOdometricToVehicle().getValue();

        double mCreepDistance = .25;

        List<Path.Waypoint> creep_path = new ArrayList<>();
        creep_path.add(new Path.Waypoint(new Translation2d(lineRobotPose.getTranslation().getX(), 0), 20.0));
        creep_path.add(
                new Path.Waypoint(new Translation2d(lineRobotPose.getTranslation().getX() + mCreepDistance, 0), 20.0));

        // intake ball
        mSuperstructure.setWantsToRunIntake();
        mSuperstructure.deployIntake();
        runAction(new FollowPathAction(new Path(creep_path), false));

        // Go over defenses again
        List<Path.Waypoint> second_shot_path = new ArrayList<>();
        second_shot_path
                .add(new Path.Waypoint(new Translation2d(lineRobotPose.getTranslation().getX(), mCreepDistance), 85.0));
        second_shot_path.add(
                new Path.Waypoint(new Translation2d(lineRobotPose.getTranslation().getX() + mDistanceToDrive - 15, 0),
                        85.0, "START_AIM"));
        second_shot_path.add(new Path.Waypoint(
                new Translation2d(lineRobotPose.getTranslation().getX() + mDistanceToDrive, 0), 85.0));

        runAction(new ParallelAction(
                Arrays.asList(new FollowPathAction(new Path(second_shot_path), false), new SeriesAction(
                        Arrays.asList(new WaitForPathMarkerAction("START_AIM"), new StartAutoAimingAction())))));

        // Shoot 2nd ball
        runAction(new WaitAction(.5));
        runAction(new ShootWhenReadyAction());
        mSuperstructure.setWantsToStopIntake();
    }
}
