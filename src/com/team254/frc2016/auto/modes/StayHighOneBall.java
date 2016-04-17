package com.team254.frc2016.auto.modes;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.*;
import com.team254.frc2016.subsystems.UtilityArm;
import com.team254.lib.util.Path;
import com.team254.lib.util.Translation2d;
import com.team254.lib.util.Path.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Go over the defenses in starting config
 */
public class StayHighOneBall extends AutoModeBase {

    private final boolean mShouldDriveBack;
    private final double mDistanceToDrive;

    public static final double DISTANCE_TO_DROP_ARM = 100;

    public StayHighOneBall(boolean shouldDriveBack, double distanceToDrive) {
        mShouldDriveBack = shouldDriveBack;
        mDistanceToDrive = distanceToDrive;
    }

    @Override
    protected void routine() throws AutoModeEndedException {

        List<Waypoint> first_path = new ArrayList<>();
        first_path.add(new Waypoint(new Translation2d(0, 0), 48.0));
        first_path.add(new Waypoint(new Translation2d(DISTANCE_TO_DROP_ARM, 0), 36.0, "DropArm"));
        first_path.add(new Waypoint(new Translation2d(mDistanceToDrive, 0), 48.0));

        List<Waypoint> return_path = new ArrayList<>();
        return_path.add(new Waypoint(new Translation2d(mDistanceToDrive, 0), 48.0));
        return_path.add(new Waypoint(new Translation2d(12, 0), 48.0));

        runAction(
                new ParallelAction(Arrays.asList(new FollowPathAction(new Path(first_path), false),
                        new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("DropArm"),
                                new SetArmModeAction(UtilityArm.WantedState.LOW_BAR),
                                new StartAutoAimingAction())))));

        runAction(new WaitAction(1));
        runAction(new ShootWhenReadyAction());
        runAction(new WaitAction(0.75));
        runAction(new SetArmModeAction(UtilityArm.WantedState.DRIVING));

        if (mShouldDriveBack) {
            runAction(new FollowPathAction(new Path(return_path), true));
        }
    }
}
