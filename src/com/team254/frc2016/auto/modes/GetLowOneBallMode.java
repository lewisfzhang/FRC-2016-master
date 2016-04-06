package com.team254.frc2016.auto.modes;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.*;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.UtilityArm;
import com.team254.lib.util.Path;
import com.team254.lib.util.Rotation2d;
import com.team254.lib.util.Translation2d;
import com.team254.lib.util.Path.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Get's into portcullis/low bar position before driving through the defense.
 */
public class GetLowOneBallMode extends AutoModeBase {

    private final boolean mShouldDriveBack;
    private final double mDistanceToDrive;

    private static final double DISTANCE_TO_DROP_INTAKE = 12;

    public GetLowOneBallMode(boolean shouldDriveBack, double distanceToDrive) {
        mShouldDriveBack = shouldDriveBack;  // TODO currently ignored
        mDistanceToDrive = distanceToDrive;
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        List<Waypoint> first_path = new ArrayList<>();
        first_path.add(new Waypoint(new Translation2d(0, 0), 48.0));
        first_path.add(new Waypoint(new Translation2d(DISTANCE_TO_DROP_INTAKE, 0), 48.0, "DropIntake"));
        first_path.add(new Waypoint(new Translation2d(AutoModeUtils.DISTANCE_TO_POP_HOOD, 0), 48.0, "PopHood"));
        first_path.add(new Waypoint(new Translation2d(mDistanceToDrive, 0), 48.0));
        
        // Get low and wait a minimum of 1.5 seconds
        runAction(new ParallelAction(Arrays.asList(new GetLowAction(), new WaitAction(1.5))));

        runAction(new ParallelAction(
                Arrays.asList(new FollowPathAction(new Path(first_path), false),
                        new SeriesAction(Arrays.asList(new WaitForPathMarkerAction("DropIntake"),
                                new DeployIntakeAction(), new WaitForPathMarkerAction("PopHood"), new StartAutoAimingAction())))));

        runAction(new WaitAction(1));
        runAction(new ShootWhenReadyAction());
        runAction(new WaitAction(0.75));
        runAction(new SetArmModeAction(UtilityArm.WantedState.DRIVING));
    }
}
