package com.team254.frc2016;

import com.team254.lib.util.ConstantsBase;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.Solenoid;

public class Constants extends ConstantsBase {
    public static double kCenterOfTargetHeight = 89.0; // inches

    // Pose of the turret frame w.r.t. the vehicle frame
    public static double kTurretXOffset = -7.376;
    public static double kTurretYOffset = 0.0;
    public static double kTurretAngleOffsetDegrees = 0.0;

    // Pose of the camera frame w.r.t. the turret frame
    public static double kCameraXOffset = -6.454;
    public static double kCameraYOffset = 0.0;
    public static double kCameraZOffset = 19.75;
    public static double kCameraPitchAngleDegrees = 33.67;
    public static double kCameraYawAngleDegrees = -0.5;
    public static double kCameraDeadband = 0.01;

    // Wheels
    public static double kDriveWheelDiameterInches = 7.07; // Measured on
                                                           // 4/5/2016
    public static double kTrackLengthInches = 8.265;
    public static double kTrackWidthInches = 23.8;
    public static double kTrackEffectiveDiameter = (kTrackWidthInches * kTrackWidthInches
            + kTrackLengthInches * kTrackLengthInches) / kTrackWidthInches;
    public static double kTrackScrubFactor = .5; // Should be <= 1.0...tune
                                                 // experimentally

    // Drive constants
    public static double kDriveLowGearMaxSpeedInchesPerSec = 12.0 * 7.0;

    // Hood constants
    public static double kMinHoodAngle = 42.48;
    public static double kMaxHoodAngle = 71.42;
    public static double kBatterHoodAngle = 42.5;
    public static double kHoodNeutralAngle = 42.5;
    public static double kHoodMaxSafeAngle = 43.0;
    public static double kHoodOnTargetTolerance = 0.4;
    public static double kHoodGearReduction = 12.0 / 644.0;

    // Turret constants
    public static double kHardMaxTurretAngle = 109.5;
    public static double kHardMinTurretAngle = -116.5;
    public static double kSoftMaxTurretAngle = 108.0;
    public static double kSoftMinTurretAngle = -115.0;
    public static double kTurretSafeTolerance = 2.0;
    public static double kTurretOnTargetTolerance = 1.0;
    public static double kTurretRotationsPerTick = 14.0 / 50.0 * 14.0 / 322.0;

    // Flywheel constants
    public static double kFlywheelOnTargetTolerance = 100.0;
    public static double kFlywheelGoodBallRpmSetpoint = 5800.0;
    public static double kFlywheelBadBallRpmSetpoint = kFlywheelGoodBallRpmSetpoint;

    // Auto aiming/shooter constants
    public static double kAutoAimMinRange = 10.0;
    public static double kAutoAimMaxRange = 220.0;
    public static double kAutoShootMaxDriveSpeed = 18.0;
    public static double kAutoAimPredictionTime = 0.25;
    public static double kShooterVelocityInchesPerSec = 576.0;
    public static int kAutoAimMinConsecutiveCyclesOnTarget = 3;
    public static double kShootActuationTime = 0.75;
    public static double kHoodUnstowToFlywheelSpinTime = 0.4;
    public static double kLoadingTime = 0.5;
    public static double kStowingOverrideTime = 2.0;

    // Goal tracker constants
    public static double kMaxGoalTrackAge = 0.3;
    public static double kMaxTrackerDistance = 18.0;
    public static double kCameraFrameRate = 30.0;
    public static double kTrackReportComparatorStablityWeight = 1.0;
    public static double kTrackReportComparatorAgeWeight = 1.0;
    public static double kTrackReportComparatorSwitchingWeight = 3.0;
    public static double kTrackReportComparatorDistanceWeight = 2.0; // TODO

    public static int kAndroidAppTcpPort = 8254;

    public static double kLooperDt = 0.01;

    // CONTROL LOOP GAINS

    // PID gains for hood position loop
    // Units: error is degrees of hood rotation. Max output is +/- 1.0.
    // Loop runs at 100Hz
    public static double kHoodKp = 0.1;
    public static double kHoodKi = 0.0;
    public static double kHoodKd = 0.0;
    public static double kHoodDeadband = 0.3; // degrees

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    public static double kDriveVelocityKp = 0.5;
    public static double kDriveVelocityKi = 0.0;
    public static double kDriveVelocityKd = 3.0;
    public static double kDriveVelocityKf = 0.5;
    public static int kDriveVelocityIZone = 0;
    public static double kDriveVelocityRampRate = 0.0;
    public static int kDriveVelocityAllowableError = 0;

    // PID gains for drive base lock loop
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    public static double kDriveBaseLockKp = 0.5;
    public static double kDriveBaseLockKi = 0;
    public static double kDriveBaseLockKd = 0;
    public static double kDriveBaseLockKf = 0;
    public static int kDriveBaseLockIZone = 0;
    public static double kDriveBaseLockRampRate = 0;
    public static int kDriveBaseLockAllowableError = 10;

    // PID gains for constant heading velocity control
    // Units: Error is degrees. Output is inches/second difference to
    // left/right.
    public static double kDriveHeadingVelocityKp = 4.0; // 6.0;
    public static double kDriveHeadingVelocityKi = 0.0;
    public static double kDriveHeadingVelocityKd = 50.0;

    // Path following constants
    public static double kPathFollowingLookahead = 30.0; // inches
    public static double kPathFollowingMaxVel = 84.0; // inches/sec
    public static double kPathFollowingMaxAccel = 65.0; // inches/sec^2

    // PID gains for turret position loop
    // Units: error is 4096 counts/rev. Max output is +/- 1023 units.
    public static double kTurretKp = 0.7;
    public static double kTurretKi = 0.0;
    public static double kTurretKd = 30.0;
    public static double kTurretKf = 0;
    public static int kTurretIZone = (int) (1023.0 / kTurretKp);
    public static double kTurretRampRate = 0;
    public static int kTurretAllowableError = 100;

    // PID gains for flywheel velocity loop
    // Units: error is (4096 counts/rev)/100ms. Max output is +/- 1023 units.
    public static double kFlywheelKp = 0.12;
    public static double kFlywheelKi = 0.0;
    public static double kFlywheelKd = 0.5;
    public static double kFlywheelKf = 0.014;
    public static int kFlywheelIZone = (int) (1023.0 / kFlywheelKp);
    public static double kFlywheelRampRate = 0;
    public static int kFlywheelAllowableError = 0;

    // Utility arm time delays, all in seconds
    public static double kUtilityArmDropTime = 0.7;
    public static double kUtilityArmRaiseTime = 1.0;
    public static double kUtilityArmHardStopsMoveForRaiseArmDelay = 0.5;

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
    public static final int kIntakeTalonId = 13;
    public static final int kFixedRollerTalonId = 14;
    public static final int kHoodRollerTalonId = 6;
    public static final int kHangerMasterTalonId = 5;
    public static final int kHangerSlaveTalonId = 7;

    // SOLENOIDS
    public static final int kShifterSolenoidId = 11; // PCM 1, Solenoid 3
    public static final int kHoodStowSolenoidId = 1; // PCM 0, Solenoid 1
    public static final int kIntakeSolenoidId = 10; // PCM 1, Solenoid 2
    public static final int kShooterSolenoidId = 0; // PCM 0, Solenoid 0

    public static final int kArmLiftSolenoidId = 9; // PCM 1, Solenoid 1
    public static final int kAdjustableHardStopSolenoidId = 2; // PCM 0,
                                                               // Solenoid 2
    public static final int kBrakeSolenoidId = 3; // PCM 0, Solenoid 3
    public static final int kHookReleaseSolenoidId = 4; // PCM 0, Solenoid 4

    // Analog Inputs
    public static int kHaveBallSensorAnalogId = 1;
    public static int kBallReadyAnalogId = 2;

    /**
     * Make an {@link Solenoid} instance for the single-number ID of the
     * solenoid
     * 
     * @param solenoidId
     *            One of the kXyzSolenoidId constants
     */
    public static Solenoid makeSolenoidForId(int solenoidId) {
        return new Solenoid(solenoidId / 8, solenoidId % 8);
    }

    // DIGITAL IO
    public static final int kHoodEncoderDIO = 9;
    public static final int kLineSensor1DIO = 1;
    public static final int kLineSensor2DIO = 2;

    // PWM
    public static final int kSensorSideServoPWM = 0;
    public static final int kOppositeSideServoPWM = 1;
    public static final int kSneakyServoPWM = 3;
    public static final int kTestServoPWM = 9;

    @Override
    public String getFileLocation() {
        return "~/constants.txt";
    }

    // Shooter Operational consts
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodAutoAimMap = new InterpolatingTreeMap<>();

    static {
        kHoodAutoAimMap.put(new InterpolatingDouble(62.0), new InterpolatingDouble(42.5));
        kHoodAutoAimMap.put(new InterpolatingDouble(67.0), new InterpolatingDouble(42.5));
        kHoodAutoAimMap.put(new InterpolatingDouble(77.0), new InterpolatingDouble(44.8));
        kHoodAutoAimMap.put(new InterpolatingDouble(85.0), new InterpolatingDouble(48.0));
        kHoodAutoAimMap.put(new InterpolatingDouble(92.0), new InterpolatingDouble(50.0));
        kHoodAutoAimMap.put(new InterpolatingDouble(100.0), new InterpolatingDouble(51.0));
        kHoodAutoAimMap.put(new InterpolatingDouble(110.0), new InterpolatingDouble(53.0));
        kHoodAutoAimMap.put(new InterpolatingDouble(125.0), new InterpolatingDouble(54.5));
        kHoodAutoAimMap.put(new InterpolatingDouble(140.0), new InterpolatingDouble(56.0));
        kHoodAutoAimMap.put(new InterpolatingDouble(155.0), new InterpolatingDouble(57.0));
        kHoodAutoAimMap.put(new InterpolatingDouble(168.0), new InterpolatingDouble(57.5));
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kFlywheelAutoAimMap = new InterpolatingTreeMap<>();

    static {
        kFlywheelAutoAimMap.put(new InterpolatingDouble(Constants.kAutoAimMinRange),
                new InterpolatingDouble(Constants.kFlywheelGoodBallRpmSetpoint));
        kFlywheelAutoAimMap.put(new InterpolatingDouble(Constants.kAutoAimMaxRange),
                new InterpolatingDouble(Constants.kFlywheelGoodBallRpmSetpoint));
    }

    static {
        new Constants().loadFromFile();
    }
}
