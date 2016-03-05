package com.team254.frc2016;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Controls the interactive elements of smartdashboard.
 *
 * Keeps the network tables keys in one spot and enforces auton mode invariants.
 */
public class SmartDashboardInteractions {
    private static final String HOOD_TUNING_MODE = "Hood Tuning Mode";
    private static final String OUTPUT_TO_SMART_DASHBOARD = "Output To SmartDashboard";
    private static final String BALLS_WORN = "Balls Worn?";

    public void initWithDefaults() {
        SmartDashboard.putBoolean(HOOD_TUNING_MODE, false);
        SmartDashboard.putBoolean(OUTPUT_TO_SMART_DASHBOARD, false);
        SmartDashboard.putBoolean(BALLS_WORN, false);
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
}
