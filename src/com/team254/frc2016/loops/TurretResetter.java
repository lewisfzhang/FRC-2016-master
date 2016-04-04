package com.team254.frc2016.loops;

import com.team254.frc2016.subsystems.Superstructure;
import com.team254.frc2016.subsystems.Turret;

public class TurretResetter implements Loop {
    Superstructure mSuperstructure = Superstructure.getInstance();
    Turret mTurret = Superstructure.getInstance().getTurret();

    @Override
    public void onStart() {
        // no-op
    }

    @Override
    public void onLoop() {
        if (mTurret.getForwardLimitSwitch()) {
            mSuperstructure.resetTurretAtMax();
        } else if (mTurret.getReverseLimitSwitch()) {
            mSuperstructure.resetTurretAtMin();
        }
    }

    @Override
    public void onStop() {
        // no-op
    }

}
