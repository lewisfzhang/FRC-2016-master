package com.team254.frc2016.auto.modes;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;

/**
 * "Oh shit everything is broken and we shouldn't run auton" Mode
 */
public class StandStillMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Starting Stand Still Mode... Done!");
    }
}
