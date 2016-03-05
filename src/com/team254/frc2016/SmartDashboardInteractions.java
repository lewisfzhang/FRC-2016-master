package com.team254.frc2016;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.modes.OneBallMode;
import com.team254.frc2016.auto.modes.OneBallThenReturnMode;
import com.team254.frc2016.auto.modes.StandStillMode;
import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.Callable;

/**
 * Controls the interactive elements of smartdashboard.
 *
 * Keeps the network tables keys in one spot and enforces auton mode invariants.
 */
public class SmartDashboardInteractions {

    private static final String HOOD_TUNING_MODE = "Hood Tuning Mode";
    private static final String OUTPUT_TO_SMART_DASHBOARD = "Output To SmartDashboard";
    private static final String BALLS_WORN = "Balls Worn?";
    private static final String AUTON_MODE = "Auton Mode";

    private SendableChooser mAutonChooser;

    public void initWithDefaults() {
        SmartDashboard.putBoolean(HOOD_TUNING_MODE, false);
        SmartDashboard.putBoolean(OUTPUT_TO_SMART_DASHBOARD, false);
        SmartDashboard.putBoolean(BALLS_WORN, false);

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

    public boolean areBallsWorn() {
        return SmartDashboard.getBoolean(BALLS_WORN, false);
    }

    public AutoModeBase getSelectedAutonMode() {
        AutonOption selected = (AutonOption) mAutonChooser.getSelected();
        if (selected == null) {
            System.out.println("WARNING: Got null auton mode");
            selected = AutonOption.STAND_STILL;
        }
        try {
            return selected.modeFactory.call();
        } catch (Exception e) {
            // Should never happen
            e.printStackTrace();
            return new StandStillMode();
        }
    }

    /**
     * I don't trust {@link SendableChooser} to manage {@link AutoModeBase}
     * objects directly, so use this enum to project us from WPILIb.
     */
    enum AutonOption {
        ONE_BALL_MODE("One Ball", () -> new OneBallMode(Shooter.getInstance())),
        ONE_BALL_MODE_WITH_RETURN(
                "One Ball With Return",
                () -> new OneBallThenReturnMode(Drive.getInstance(), Shooter.getInstance())),
        STAND_STILL("Stand Still", StandStillMode::new);

        public final String name;
        public final Callable<AutoModeBase> modeFactory;

        AutonOption(String name, Callable<AutoModeBase> modeFactory) {
            this.name = name;
            this.modeFactory = modeFactory;
        }
    }
}
