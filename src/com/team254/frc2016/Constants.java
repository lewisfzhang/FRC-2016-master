package com.team254.frc2016;

import com.team254.lib.util.ConstantsBase;

public class Constants extends ConstantsBase {
    // Pose of the turret frame w.r.t. the vehicle frame
    public static double kTurretXOffset = -10.0;
    public static double kTurretYOffset = 0.0;
    public static double kTurretAngleOffsetDegrees = 0.0;

    // Pose of the camera frame w.r.t. the turret frame
    public static double kCameraXOffset = -5.0;
    public static double kCameraYOffset = 0.0;
    public static double kCameraAngleOffsetDegrees = 0.0;

    public static int kVisionUDPPort = 5254;

    // Do not change anything after this line!
    // Talons
    public static int kTurretTalonId = 9;
    public static int kLeftDriveMasterId = 11;
    public static int kLeftDriveSlaveId = 12;
    public static int kRightDriveMasterId = 4;
    public static int kRightDriveSlaveId = 3;
    

    @Override
    public String getFileLocation() {
        return "~/constants.txt";
    }

    static {
        new Constants().loadFromFile();
    }
}
