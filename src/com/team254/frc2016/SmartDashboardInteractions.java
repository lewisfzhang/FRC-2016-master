package com.team254.frc2016;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.DriveStraightAction;
import com.team254.frc2016.auto.actions.FollowPathAction;
import com.team254.frc2016.auto.modes.*;
import com.team254.lib.util.Path;
import com.team254.lib.util.Path.Waypoint;
import com.team254.lib.util.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

/**
 * Controls the interactive elements of smartdashboard.
 *
 * Keeps the network tables keys in one spot and enforces auton mode invariants.
 */
public class SmartDashboardInteractions {

    private static final String HOOD_TUNING_MODE = "Hood Tuning Mode";
    private static final String OUTPUT_TO_SMART_DASHBOARD = "Output To SmartDashboard";
    private static final String SHOULD_RESET_UTILITY_ARM = "Robot in Start Position";
    private static final String AUTON_MODE = "Auton Mode";
    private static final String AUTON_LANE = "Auton Lane";

    private SendableChooser mAutonModeChooser;
    private SendableChooser mAutonLaneChooser;

    public void initWithDefaults() {
        SmartDashboard.putBoolean(HOOD_TUNING_MODE, false);
        SmartDashboard.putBoolean(OUTPUT_TO_SMART_DASHBOARD, true);
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
        return createAutoMode((AutonOption) mAutonModeChooser.getSelected(),
                (AutonLane) mAutonLaneChooser.getSelected());
    }

    /**
     * I don't trust {@link SendableChooser} to manage {@link AutoModeBase}
     * objects directly, so use this enum to project us from WPILIb.
     */
    enum AutonOption {
        STAY_HIGH_ONE_BALL_DRIVE_BACK("No Drop Drive Back"), //
        STAY_HIGH_ONE_BALL("No Drop Stay"), //
        GET_LOW_ONE_BALL("Portcullis"), //
        CDF_ONE_BALL("CDF - Stop"), //
        CDF_COME_BACK_LEFT("CDF - Come back left"), //
        CDF_COME_BACK_RIGHT("CDF - Come back right"), //
        TWO_BALL("Two Ball"), //
        STAND_STILL("Stand Still"),
        DRIVE_1_FOOT("Drive 1 Foot");

        public final String name;

        AutonOption(String name) {
            this.name = name;
        }
    }

    enum AutonLane {
        LANE_1("Lane 1 (low bar)", 170), //
        LANE_2("Lane 2", 230), //
        LANE_3("Lane 3", 160), //
        LANE_4("Lane 4", 155), //
        LANE_5("Lane 5", 170);

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
            return new StayHighOneBall(false, autonLane.distanceToDrive);
        case STAY_HIGH_ONE_BALL_DRIVE_BACK:
            return new StayHighOneBall(true, autonLane.distanceToDrive);
        case GET_LOW_ONE_BALL:
            return new GetLowOneBallMode(false, autonLane.distanceToDrive);
        case CDF_ONE_BALL:
            return new ShovelTheFriesMode(autonLane.distanceToDrive, false, false);
        case CDF_COME_BACK_LEFT:
            return new ShovelTheFriesMode(autonLane.distanceToDrive, true, false);
        case CDF_COME_BACK_RIGHT:
            return new ShovelTheFriesMode(autonLane.distanceToDrive, true, true);
        case TWO_BALL:
            return new TwoBallMode(autonLane.distanceToDrive);
        case DRIVE_1_FOOT:
            return new AutoModeBase() {
                @Override
                protected void routine() throws AutoModeEndedException {
                    ArrayList<Waypoint> path = new ArrayList<>();
                    path.add(new Waypoint(new Translation2d(0, 0), 12.0));
                    path.add(new Waypoint(new Translation2d(12, 0), 12.0));

                    runAction(new FollowPathAction(new Path(path), false));
                }
            };
        case STAND_STILL: // fallthrough
        default:
            System.out.println("ERROR: unexpected auto mode: " + autonOption);
            return new StandStillMode();
        }
    }
}
