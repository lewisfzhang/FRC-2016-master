package com.team254.frc2016.loops;

import com.team254.frc2016.subsystems.Shooter;
import com.team254.frc2016.subsystems.Turret;

public class TurretResetter implements Loop {
    Shooter mShooter = Shooter.getInstance();
    Turret mTurret = Shooter.getInstance().getTurret();

    @Override
    public void onStart() {
        // no-op
    }

    @Override
    public void onLoop() {
        if (mTurret.getForwardLimitSwitch()) {
            mShooter.resetTurretAtMax();
        } else if (mTurret.getReverseLimitSwitch()) {
            mShooter.resetTurretAtMin();
        }
    }

    @Override
    public void onStop() {
        // no-op
    }

}
