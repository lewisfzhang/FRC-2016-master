package com.team254.frc2016;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.modes.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Controls the interactive elements of smartdashboard.
 *
 * Keeps the network tables keys in one spot and enforces auton mode invariants.
 */
public class SmartDashboardInteractions {

    private static final String HOOD_TUNING_MODE = "Hood Tuning Mode";
    private static final String OUTPUT_TO_SMART_DASHBOARD = "Output To SmartDashboard";
    private static final String IS_AUTON_BALL_BAD = "Auton Ball Worn?";
    private static final String SHOULD_RESET_UTILITY_ARM = "Robot in Start Position";
    private static final String AUTON_MODE = "Auton Mode";
    private static final String AUTON_LANE = "Auton Lane";

    private SendableChooser mAutonModeChooser;
    private SendableChooser mAutonLaneChooser;

    public void initWithDefaults() {
        SmartDashboard.putBoolean(HOOD_TUNING_MODE, false);
        SmartDashboard.putBoolean(OUTPUT_TO_SMART_DASHBOARD, true);
        SmartDashboard.putBoolean(IS_AUTON_BALL_BAD, false);
        SmartDashboard.putBoolean(SHOULD_RESET_UTILITY_ARM, false);

        mAutonModeChooser = new SendableChooser();
        // first entry will become default
        for (AutonOption autonOption : AutonOption.values()) {
            mAutonModeChooser.addObject(autonOption.name, autonOption);
        }
        SmartDashboard.putData(AUTON_MODE, mAutonModeChooser);

        mAutonLaneChooser = new SendableChooser();
        for (AutonLane autonLane : AutonLane.values()) {
            mAutonLaneChooser.addObject(autonLane.name, autonLane);
        }
        SmartDashboard.putData(AUTON_LANE, mAutonLaneChooser);
    }

    public boolean isInHoodTuningMode() {
        return SmartDashboard.getBoolean(HOOD_TUNING_MODE, false);
    }

    public boolean shouldLogToSmartDashboard() {
        return SmartDashboard.getBoolean(OUTPUT_TO_SMART_DASHBOARD, true);
    }

    public boolean shouldResetUtilityArm() {
        return SmartDashboard.getBoolean(SHOULD_RESET_UTILITY_ARM, false);
    }

    public void clearUtilityArmResetState() {
        SmartDashboard.putBoolean(SHOULD_RESET_UTILITY_ARM, false);
    }

    public AutoModeBase getSelectedAutonMode() {
        return createAutoMode(
                (AutonOption) mAutonModeChooser.getSelected(),
                (AutonLane) mAutonLaneChooser.getSelected());
    }

    /**
     * I don't trust {@link SendableChooser} to manage {@link AutoModeBase}
     * objects directly, so use this enum to project us from WPILIb.
     */
    enum AutonOption {
        STAY_HIGH_ONE_BALL_DRIVE_BACK("No Drop Drive Back"),
        STAY_HIGH_ONE_BALL("No Drop Stay"),
        GET_LOW_ONE_BALL("Portcullis"),
        CDF_ONE_BALL("CDF"),
        STAND_STILL("Stand Still");

        public final String name;

        AutonOption(String name) {
            this.name = name;
        }
    }

    enum AutonLane {
        LANE_1("Lane 1 (low bar)", 240),
        LANE_2("Lane 2", 245),
        LANE_3("Lane 3", 200),
        LANE_4("Lane 4", 195),
        LANE_5("Lane 5", 230);

        public final String name;
        public final double distanceToDrive;
        AutonLane(String name, double distanceToDrive) {
            this.name = name;
            this.distanceToDrive = distanceToDrive;
        }
    }

    private AutoModeBase createAutoMode(AutonOption autonOption, AutonLane autonLane) {
        switch (autonOption) {
        case STAY_HIGH_ONE_BALL:
            return new StayHighOneBall(isAutonBallBad(), false, autonLane.distanceToDrive);
        case STAY_HIGH_ONE_BALL_DRIVE_BACK:
            return new StayHighOneBall(isAutonBallBad(), true, autonLane.distanceToDrive);
        case GET_LOW_ONE_BALL:
            return new GetLowOneBallMode(isAutonBallBad(), false, autonLane.distanceToDrive);
        case CDF_ONE_BALL:
            return new ShovelTheFriesMode(autonLane.distanceToDrive);
        case STAND_STILL: // fallthrough
        default:
            System.out.println("ERROR: unexpected auto mode: " + autonOption);
            return new StandStillMode();
        }
    }

    private boolean isAutonBallBad() {
        return SmartDashboard.getBoolean(IS_AUTON_BALL_BAD, false);
    }
}
