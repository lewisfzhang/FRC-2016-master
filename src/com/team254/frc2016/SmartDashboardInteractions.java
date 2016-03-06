package com.team254.frc2016;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.modes.OneBallMode;
import com.team254.frc2016.auto.modes.OneBallThenReturnMode;
import com.team254.frc2016.auto.modes.StandStillMode;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Shooter;
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

    private SendableChooser mAutonChooser;

    public void initWithDefaults() {
        SmartDashboard.putBoolean(HOOD_TUNING_MODE, false);
        SmartDashboard.putBoolean(OUTPUT_TO_SMART_DASHBOARD, false);
        SmartDashboard.putBoolean(IS_AUTON_BALL_BAD, false);
        SmartDashboard.putBoolean(SHOULD_RESET_UTILITY_ARM, false);

        mAutonChooser = new SendableChooser();
        // first entry will become default
        for (AutonOption autonOption : AutonOption.values()) {
            mAutonChooser.addObject(autonOption.name, autonOption);
        }
        SmartDashboard.putData(AUTON_MODE, mAutonChooser);
    }

    public boolean isInHoodTuningMode() {
        return SmartDashboard.getBoolean(HOOD_TUNING_MODE, false);
    }

    public boolean shouldLogToSmartDashboard() {
        return SmartDashboard.getBoolean(OUTPUT_TO_SMART_DASHBOARD, false);
    }

    public boolean shouldResetUtilityArm() {
        return SmartDashboard.getBoolean(SHOULD_RESET_UTILITY_ARM, false);
    }

    public void clearUtilityArmResetState() {
        SmartDashboard.putBoolean(SHOULD_RESET_UTILITY_ARM, false);
    }

    public AutoModeBase getSelectedAutonMode() {
        AutonOption selected = (AutonOption) mAutonChooser.getSelected();
        return createAutoMode(selected);
    }

    /**
     * I don't trust {@link SendableChooser} to manage {@link AutoModeBase}
     * objects directly, so use this enum to project us from WPILIb.
     */
    enum AutonOption {
        ONE_BALL_MODE("One Ball"),
        ONE_BALL_MODE_WITH_RETURN("One Ball With Return"),
        STAND_STILL("Stand Still");

        public final String name;

        AutonOption(String name) {
            this.name = name;
        }
    }

    private AutoModeBase createAutoMode(AutonOption autonOption) {
        switch (autonOption) {
            case ONE_BALL_MODE:
                return new OneBallMode(Shooter.getInstance(), isAutonBallBad());
            case ONE_BALL_MODE_WITH_RETURN:
                return new OneBallThenReturnMode(
                        Drive.getInstance(),
                        Shooter.getInstance(),
                        isAutonBallBad());
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
