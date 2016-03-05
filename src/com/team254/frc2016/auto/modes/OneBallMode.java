package com.team254.frc2016.auto.modes;

import com.team254.frc2016.auto.AutoModeBase;
import com.team254.frc2016.auto.AutoModeEndedException;
import com.team254.frc2016.auto.actions.DriveThenAimAction;
import com.team254.frc2016.auto.actions.WaitAction;
import com.team254.frc2016.subsystems.Shooter;

/**
 * The typical one ball auto mode.
 */
public class OneBallMode extends AutoModeBase {

    private final Shooter mShooter;

    public OneBallMode(Shooter shooter) {
        mShooter = shooter;
    }

    public void doOneBallRoutine() throws AutoModeEndedException {
        runAction(new DriveThenAimAction(125, 100, 270, 45));
        runAction(new WaitAction(1));
        // TODO: should this be fireWhenReady()?
        mShooter.setWantsToFireNow();
        runAction(new WaitAction(0.75));
        mShooter.setWantedState(Shooter.WantedState.WANT_TO_STOW);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Starting");
        doOneBallRoutine();
        System.out.println("Ending");
    }
}
