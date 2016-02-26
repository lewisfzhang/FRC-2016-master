package com.team254.frc2016;

import com.team254.lib.util.ConstantsBase;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

public class Constants extends ConstantsBase {
    public static double kCenterOfTargetHeight = 89.0; // inches

    // Pose of the turret frame w.r.t. the vehicle frame
    public static double kTurretXOffset = -7.376;
    public static double kTurretYOffset = 0.0;
    public static double kTurretAngleOffsetDegrees = 0.0;

    // Pose of the camera frame w.r.t. the turret frame
    public static double kCameraXOffset = -5.535;
    public static double kCameraYOffset = 2.034;
    public static double kCameraZOffset = 20.765;
    public static double kCameraPitchAngleDegrees = 30.0;
    public static double kCameraAngleOffsetDegrees = 2.0;

    // Wheel diameter
    public static double kDriveWheelDiameterInches = 7.12; // Measured on 2/5/16

    // Hood constants
    public static double kMinHoodAngle = 26.0; // TODO tune this
    public static double kMaxHoodAngle = 59.0; // TODO tune this
    public static double kBatterHoodAngle = 28.0;
    public static double kHoodNeutralAngle = 42.5;
    public static double kHoodMaxSafeAngle = 45.0;
    public static double kHoodOnTargetTolerance = 0.5;
    public static double kHoodGearReduction = 12.0 / 708.0; // TODO check this
                                                            // with Colin

    // Turret constants
    public static double kHardMaxTurretAngle = 109.5;
    public static double kHardTurretAngle = -116.5;
    public static double kSoftMaxTurretAngle = 108.0;
    public static double kSoftMinTurretAngle = -115.0;
    public static double kTurretSafeTolerance = 2.0;
    public static double kTurretOnTargetTolerance = 1.0;
    public static double kTurretGearReduction = 14.0 / 50.0 * 14.0 / 322.0;

    // Flywheel constants
    public static double kFlywheelOnTargetTolerance = 100.0;
    public static double kFlywheelBatterRpmSetpoint = 6000.0;
    public static double kFlywheelAutoAimNominalRpmSetpoint = 6000.0;

    // Auto aiming/shooter constants
    public static double kAutoAimRangeHysteresis = 100.0;
    public static double kAutoAimMinRange = 10.0;
    public static double kAutoAimMaxRange = 200.0;
    public static double kShootActuationTime = 0.5;
    public static double kHoodUnstowToFlywheelSpinTime = 0.5;

    // Goal tracker constants
    public static double kMaxGoalTrackAge = 0.5;
    public static double kMaxTrackerDistance = 12.0;
    public static double kCameraFrameRate = 30.0;
    public static double kTrackReportComparatorStablityWeight = 1.0;
    public static double kTrackReportComparatorAgeWeight = 1.0;
    public static double kTrackReportComparatorSwitchingWeight = 3.0;

    public static int kAndroidAppTcpPort = 8254;

    // CONTROL LOOP GAINS

    // PID gains for hood position loop
    // Units: error is degrees of hood rotation. Max output is +/- 1.0.
    // Loop runs at 100Hz
    public static double kHoodKp = 0.05;
    public static double kHoodKi = 0.0001;
    public static double kHoodKd = 0.0;

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    public static double kDriveVelocityKp = 0.6;
    public static double kDriveVelocityKi = 0;
    public static double kDriveVelocityKd = 6.0;
    public static double kDriveVelocityKf = 1.0;
    public static int kDriveVelocityIZone = 0;
    public static double kDriveVelocityRampRate = 0;
    public static int kDriveVelocityAllowableError = 0;

    // PID gains for drive base lock loop
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    public static double kDriveBaseLockKp = 1.0;
    public static double kDriveBaseLockKi = 0;
    public static double kDriveBaseLockKd = 0;
    public static double kDriveBaseLockKf = 0;
    public static int kDriveBaseLockIZone = 0;
    public static double kDriveBaseLockRampRate = 0;
    public static int kDriveBaseLockAllowableError = 10;

    // PID gains for constant heading velocity control
    // Units: Error is degrees. Output is inches/second difference to
    // left/right.
    public static double kDriveHeadingVeloctyKp = 6.0;
    public static double kDriveHeadingVeloctyKi = 0.0;
    public static double kDriveHeadingVeloctyKd = 0.0;

    // PID gains for turret position loop
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    public static double kTurretKp = 0.5;
    public static double kTurretKi = 0.005;
    public static double kTurretKd = 30.0;
    public static double kTurretKf = 0;
    public static int kTurretIZone = (int) (1023.0 / kTurretKp);
    public static double kTurretRampRate = 0;
    public static int kTurretAllowableError = 0;

    // PID gains for flywheel velocity loop
    // Units: error is (4096 counts/rev)/100ms. Max output is +/- 1023 units.
    public static double kFlywheelKp = 0.075;
    public static double kFlywheelKi = 0.0;
    public static double kFlywheelKd = 0.5;
    public static double kFlywheelKf = 0.015;
    public static int kFlywheelIZone = 0;
    public static double kFlywheelRampRate = 0;
    public static int kFlywheelAllowableError = 0;

    // Do not change anything after this line!
    // Port assignments should match up with the spreadsheet here:
    // https://docs.google.com/spreadsheets/d/1O2Szvp3cC3gO2euKjqhdmpsyd54t6eB2t9jzo41G2H4
    // Talons
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)
    // TALONS
    public static final int kTurretTalonId = 10;
    public static final int kLeftDriveMasterId = 11;
    public static final int kLeftDriveSlaveId = 12;
    public static final int kRightDriveMasterId = 4;
    public static final int kRightDriveSlaveId = 3;
    public static final int kShooterMasterId = 1;
    public static final int kShooterSlaveId = 2;
    public static final int kIntakeTalonId = 7;
    public static final int kFixedRollerTalonId = 8;

    // SOLENOIDS
    public static final int kShifterSolenoidId = 11; // PCM 1, Solenoid 3
    public static final int kHoodStowSolenoidId = 1; // PCM 0, Solenoid 1
    public static final int kIntakeSolenoidId = 10; // PCM 1, Solenoid 2
    public static final int kShooterSolenoidId = 0; // PCM 0, Solenoid 0

    // DIGITAL IO
    public static final int kHoodEncoderDIO = 9;

    // PWM
    public static final int kSensorSideServoPWM = 0;
    public static final int kOppositeSideServoPWM = 1;

    @Override
    public String getFileLocation() {
        return "~/constants.txt";
    }

    // Shooter Operational consts
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodAutoAimMapWornBalls = new InterpolatingTreeMap<>();

    static {
        kHoodAutoAimMapWornBalls.put(new InterpolatingDouble(56.0), new InterpolatingDouble(41.5));
        kHoodAutoAimMapWornBalls.put(new InterpolatingDouble(57.0), new InterpolatingDouble(42.0));
        kHoodAutoAimMapWornBalls.put(new InterpolatingDouble(70.0), new InterpolatingDouble(47.0));
        kHoodAutoAimMapWornBalls.put(new InterpolatingDouble(75.0), new InterpolatingDouble(49.0));
        kHoodAutoAimMapWornBalls.put(new InterpolatingDouble(82.0), new InterpolatingDouble(50.0));
        kHoodAutoAimMapWornBalls.put(new InterpolatingDouble(90.0), new InterpolatingDouble(54.0));
        kHoodAutoAimMapWornBalls.put(new InterpolatingDouble(98.0), new InterpolatingDouble(55.0));
        kHoodAutoAimMapWornBalls.put(new InterpolatingDouble(113.0), new InterpolatingDouble(56.5));
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodAutoAimMapNewBalls = new InterpolatingTreeMap<>();

    static {
        // TODO tune these
        kHoodAutoAimMapNewBalls.put(new InterpolatingDouble(56.0), new InterpolatingDouble(41.5));
        kHoodAutoAimMapNewBalls.put(new InterpolatingDouble(57.0), new InterpolatingDouble(42.0));
        kHoodAutoAimMapNewBalls.put(new InterpolatingDouble(70.0), new InterpolatingDouble(47.0));
        kHoodAutoAimMapNewBalls.put(new InterpolatingDouble(75.0), new InterpolatingDouble(49.0));
        kHoodAutoAimMapNewBalls.put(new InterpolatingDouble(82.0), new InterpolatingDouble(50.0));
        kHoodAutoAimMapNewBalls.put(new InterpolatingDouble(90.0), new InterpolatingDouble(54.0));
        kHoodAutoAimMapNewBalls.put(new InterpolatingDouble(98.0), new InterpolatingDouble(55.0));
        kHoodAutoAimMapNewBalls.put(new InterpolatingDouble(113.0), new InterpolatingDouble(56.5));
    }

    static {
        new Constants().loadFromFile();
    }
}
