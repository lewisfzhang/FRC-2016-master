
package com.team254.frc2016;

import com.team254.frc2016.subsystems.Drive;
import com.team254.frc2016.subsystems.Turret;
import com.team254.logger.CheesyLogger;
import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

    private final CheesyLogger mCheesyLogger;
    Turret turret = Turret.getInstance();
    Drive drive = Drive.getInstance();
    CheesyDriveHelper cdh = new CheesyDriveHelper();
    ControlBoard controls = ControlBoard.getInstance();

    public Robot() {
        mCheesyLogger = CheesyLogger.makeCheesyLogger();
    }

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        mCheesyLogger.sendLogMessage("Robot Init-ed");
    }

    @Override
    public void disabledInit() {
        mCheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.DISABLED);
        drive.stop();
    }

    @Override
    public void autonomousInit() {
        mCheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.AUTO);
    }

    @Override
    public void teleopInit() {
        mCheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.TELEOP);
    }

    @Override
    public void teleopPeriodic() {
        double throttle = controls.getThrottle();
        double turn = controls.getTurn();
        drive.setOpenLoop(cdh.cheesyDrive(throttle, turn, controls.getQuickTurn()));
        mCheesyLogger.sendTimePlotPoint("joystick", "throttle", throttle);
        mCheesyLogger.sendTimePlotPoint("joystick", "turn", turn);
    }
}
