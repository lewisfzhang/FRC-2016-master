package com.team254.frc2016.auto.modes;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.*;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.UtilityArm;
import com.team254.lib.util.Path;
import com.team254.lib.util.Translation2d;
import com.team254.lib.util.Path.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ShovelTheFriesMode extends AutoModeBase {

    private double mDistanceToCDF, mDistanceToDrive;
    private boolean mShouldDriveBack;
    private boolean mComeBackRight;

    public ShovelTheFriesMode(double distanceToShootAt, boolean shouldComeBack, boolean comeBackRight) {
        mDistanceToCDF = 42.0;
        mDistanceToDrive = distanceToShootAt - mDistanceToCDF;
        mShouldDriveBack = shouldComeBack;
        mComeBackRight = comeBackRight;
    }

    @Override
    protected void routine() throws AutoModeEndedException {

        List<Waypoint> first_path = new ArrayList<>();
        first_path.add(new Waypoint(new Translation2d(0, 0), 48.0));
        first_path.add(new Waypoint(new Translation2d(mDistanceToCDF, 0), 48.0));

        List<Waypoint> second_path = new ArrayList<>();
        second_path.add(new Waypoint(new Translation2d(mDistanceToCDF, 0), 36.0));
        second_path.add(new Waypoint(new Translation2d(mDistanceToDrive, 0), 48.0));

        boolean has_first_return_path = (mDistanceToDrive < 160);
        double y_distance = (mComeBackRight ? -44 : 61);
        List<Waypoint> first_return_path = new ArrayList<>();
        List<Waypoint> second_return_path = new ArrayList<>();
        if (has_first_return_path) {
            first_return_path.add(new Waypoint(new Translation2d(mDistanceToDrive, 0), 84.0));
            // TODO different left or right distances
            first_return_path.add(new Waypoint(new Translation2d(160, y_distance), 84.0));
            first_return_path.add(new Waypoint(new Translation2d(180, y_distance), 84.0));
            second_return_path.add(new Waypoint(new Translation2d(180, y_distance), 48.0));
        } else {
            second_return_path.add(new Waypoint(new Translation2d(mDistanceToDrive, 0), 84.0));
            second_return_path.add(new Waypoint(new Translation2d(mDistanceToDrive, y_distance), 48.0));
            second_return_path.add(new Waypoint(new Translation2d(160, y_distance), 84.0));
        }
        // CDF mode causes lots of counts to be skipped, so we can safely come back this far
        second_return_path.add(new Waypoint(new Translation2d(-12, y_distance), 48.0));
        
        runAction(new FollowPathAction(new Path(first_path), false));
        runAction(new GetLowAction());
        runAction(new StartAutoAimingAction());
        runAction(new FollowPathAction(new Path(second_path), false));

        runAction(new WaitAction(1));
        runAction(new ShootWhenReadyAction());
        runAction(new WaitAction(0.75));
        runAction(new SetArmModeAction(UtilityArm.WantedState.DRIVING));

        if (mShouldDriveBack) {
            if (has_first_return_path) {
                runAction(new FollowPathAction(new Path(first_return_path), false));
            }
            runAction(new FollowPathAction(new Path(second_return_path), true));
        }
    }
}
