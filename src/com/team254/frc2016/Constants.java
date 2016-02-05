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

    public static double kDriveWheelDiameterInches = 7.7; // TODO: tune this!

    public static int kVisionUDPPort = 5254;

    // PID gains for drive velocity loop
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    // TODO: tune these!
    public static double kDriveVelocityKp = 0;
    public static double kDriveVelocityKi = 0;
    public static double kDriveVelocityKd = 0;
    public static double kDriveVelocityKf = 1023 / (15 * 12 / 100); // 15 fps
                                                                    // open loop
    public static int kDriveVelocityIZone = 0;
    public static double kDriveVelocityRampRate = 0;

    // PID gains for drive base lock loop
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    // TODO: tune these!
    public static double kDriveBaseLockKp = 0.01;
    public static double kDriveBaseLockKi = 0;
    public static double kDriveBaseLockKd = 0;
    public static double kDriveBaseLockKf = 0;
    public static int kDriveBaseLockIZone = 0;
    public static double kDriveBaseLockRampRate = 0;

    // Do not change anything after this line!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/1O2Szvp3cC3gO2euKjqhdmpsyd54t6eB2t9jzo41G2H4
    // Talons
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)
    public static final int kTurretTalonId = 10;
    public static final int kLeftDriveMasterId = 11;
    public static final int kLeftDriveSlaveId = 12;
    public static final int kRightDriveMasterId = 4;
    public static final int kRightDriveSlaveId = 3;

    public static final int kShifterSolenoidId = 1; // TODO: determine this

    @Override
    public String getFileLocation() {
        return "~/constants.txt";
    }

    static {
        new Constants().loadFromFile();
    }
}
