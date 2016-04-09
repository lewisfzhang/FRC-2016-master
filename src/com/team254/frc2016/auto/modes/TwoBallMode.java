package com.team254.frc2016.auto.modes;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.*;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Superstructure;
import com.team254.frc2016.subsystems.Superstructure.WantedState;
import com.team254.lib.util.Path;
import com.team254.lib.util.Path.Waypoint;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;

public class TwoBallMode extends AutoModeBase {
    Superstructure mSuperstructure = Superstructure.getInstance();
    Drive mDrive = Drive.getInstance();
    double mDistanceToDrive;

    public TwoBallMode(double distance_to_drive) {
        mDistanceToDrive = distance_to_drive;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Starting 2 ball auto mode...");
        List<Waypoint> first_path = new ArrayList<>();
        first_path.add(new Waypoint(new Translation2d(0, 0), 84.0));
        first_path.add(new Waypoint(new Translation2d(6, 0), 54.0));
        first_path.add(new Waypoint(new Translation2d(24, 22), 54.0));
        first_path.add(new Waypoint(new Translation2d(80, 22), 84.0));
        first_path.add(new Waypoint(new Translation2d(86, 22), 84.0, "PopHood"));
        first_path.add(new Waypoint(new Translation2d(160, 22), 84.0));

        List<Waypoint> second_path = new ArrayList<>();
        second_path.add(new Waypoint(new Translation2d(160, 22), 84.0));
        second_path.add(new Waypoint(new Translation2d(80, 22), 54.0));
        second_path.add(new Waypoint(new Translation2d(12, 27), 54.0));

        List<Waypoint> third_path = new ArrayList<>();
        third_path.add(new Waypoint(new Translation2d(12, 27), 54.0));
        third_path.add(new Waypoint(new Translation2d(80, 22), 84.0));
        third_path.add(new Waypoint(new Translation2d(86, 22), 84.0, "PopHood"));
        third_path.add(new Waypoint(new Translation2d(160, 22), 84.0));

        mSuperstructure.setWantsToRunIntake();
        mSuperstructure.deployIntake();
        runAction(new ParallelAction(Arrays.asList(new GetLowAction(), new WaitAction(0.75))));
        mSuperstructure.setWantsToStopIntake();

        runAction(
                new ParallelAction(
                        Arrays.asList(new FollowPathAction(new Path(first_path), false),
                                new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PopHood"),
                                        new StartAutoAimingAction(),
                                        new PointTurretAction(Rotation2d.fromDegrees(-32.0)))))));
        runAction(new WaitAction(0.5));
        runAction(new ShootWhenReadyAction());
        mSuperstructure.setWantedState(WantedState.WANT_TO_STOW);
        mSuperstructure.setWantsToRunIntake();
        runAction(new FollowPathAction(new Path(second_path), true));
        runAction(
                new ParallelAction(
                        Arrays.asList(new FollowPathAction(new Path(third_path), false),
                                new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("PopHood"),
                                        new StartAutoAimingAction(),
                                        new PointTurretAction(Rotation2d.fromDegrees(-32.0)))))));
        runAction(new WaitAction(0.5));
        runAction(new ShootWhenReadyAction());
        mSuperstructure.setWantedState(WantedState.WANT_TO_STOW);
        mSuperstructure.setWantsToStopIntake();

    }
}
