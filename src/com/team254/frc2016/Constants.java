package com.team254.frc2016;

import com.team254.lib.util.ConstantsBase;

public class Constants extends ConstantsBase {

    public static int kVisionUDPPort = 5254;

    // Do not change anything after this line!
    // Talons
    public static int kTurretTalonId = 9;
    public static int kLeftDriveAId = 11;
    public static int kLeftDriveBId = 12;
    public static int kRightDriveAId = 3;
    public static int kRightDriveBId = 4;

    @Override
    public String getFileLocation() {
        return "~/constants.txt";
    }

    static {
        new Constants().loadFromFile();
    }
}
