package com.team254.frc2016.auto.modes;

import com.team254.frc2016.auto.actions.Action;
import com.team254.frc2016.auto.actions.DriveStraightAction;
import com.team254.frc2016.subsystems.Drive;

public class AutoModeUtils {

    public static final double DISTANCE_TO_POP_HOOD = 140;
    public static final double FORWARD_DRIVE_VELOCITY = 30;

    public static Action makeDriveBackAction(Drive drive) {
        double driveBackDistance =
                -(drive.getLeftDistanceInches() + drive.getRightDistanceInches()) / 2.0
                        + 16.0;
        return new DriveStraightAction(driveBackDistance, -45);
    }
}
