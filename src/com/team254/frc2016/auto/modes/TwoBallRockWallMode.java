package com.team254.frc2016.auto.modes;

import com.team254.frc2016.RobotState;
import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.*;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Superstructure;
import com.team254.frc2016.subsystems.Superstructure.WantedState;
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

    public static final double DISTANCE_TO_DROP_ARM = 130;
    public static final double DISTANCE_TO_SLOW_DOWN = 40;
    public static final double DISTANCE_TO_START_AIMING = 150;
    public static final double TOTAL_DISTANCE_TO_DRIVE = 170;

    @Override
    protected void routine() throws AutoModeEndedException {
        // Make paths
        List<Path.Waypoint> first_path = new ArrayList<>();
        first_path.add(new Path.Waypoint(new Translation2d(0, 0), 120.0));
        first_path.add(new Path.Waypoint(new Translation2d(DISTANCE_TO_SLOW_DOWN, 0), 60.0));
        first_path.add(new Path.Waypoint(new Translation2d(DISTANCE_TO_DROP_ARM, 0), 120.0, "DropArm"));
        first_path.add(new Path.Waypoint(new Translation2d(DISTANCE_TO_START_AIMING, 0), 120.0, "START_AIM"));
        first_path.add(new Path.Waypoint(new Translation2d(TOTAL_DISTANCE_TO_DRIVE, 0), 120.0));

        List<Path.Waypoint> return_path = new ArrayList<>();
        return_path.add(new Path.Waypoint(new Translation2d(TOTAL_DISTANCE_TO_DRIVE, 0), 120.0));
        return_path.add(new Path.Waypoint(new Translation2d(DISTANCE_TO_DROP_ARM + 10, 0), 60.0));
        return_path.add(new Path.Waypoint(new Translation2d(48, 0), 60.0, "WatchLine"));
        return_path.add(new Path.Waypoint(new Translation2d(24, 0), 25.0));
        return_path.add(new Path.Waypoint(new Translation2d(-100, 0), 25.0));

        // Start robot actions
        mSuperstructure.setWantedState(Superstructure.WantedState.WANT_TO_KEEP_SPINNING);
        runAction(new ParallelAction(Arrays.asList(new FollowPathAction(new Path(first_path), false),
                new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("DropArm"),
                        new SetArmModeAction(UtilityArm.WantedState.LOW_BAR), new WaitForPathMarkerAction("START_AIM"),
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
        second_shot_path.add(
                new Path.Waypoint(new Translation2d(lineRobotPose.getTranslation().getX() + mCreepDistance, 0), 120.0));
        second_shot_path.add(new Path.Waypoint(
                new Translation2d(DISTANCE_TO_SLOW_DOWN + lineRobotPose.getTranslation().getX(), 0), 60.0));
        second_shot_path.add(new Path.Waypoint(
                new Translation2d(DISTANCE_TO_DROP_ARM + lineRobotPose.getTranslation().getX(), 0), 120.0, "DropArm"));
        second_shot_path.add(new Path.Waypoint(
                new Translation2d(DISTANCE_TO_START_AIMING + lineRobotPose.getTranslation().getX(), 0), 120.0,
                "START_AIM"));
        second_shot_path.add(new Path.Waypoint(
                new Translation2d(TOTAL_DISTANCE_TO_DRIVE + lineRobotPose.getTranslation().getX(), 0), 120.0));

        runAction(new ParallelAction(
                Arrays.asList(new FollowPathAction(new Path(second_shot_path), false), new SeriesAction(
                        Arrays.asList(new WaitForPathMarkerAction("START_AIM"), new StartAutoAimingAction())))));

        // Shoot 2nd ball
        runAction(new WaitAction(.5));
        runAction(new ShootWhenReadyAction());
        mSuperstructure.setWantedState(WantedState.WANT_TO_DEPLOY);
        mSuperstructure.setWantsToStopIntake();
    }
}
