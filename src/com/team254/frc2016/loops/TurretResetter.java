package com.team254.frc2016.loops;

import com.team254.frc2016.Constants;
import com.team254.frc2016.subsystems.Turret;
import com.team254.lib.util.Rotation2d;

public class TurretResetter implements Loop {
    Turret turret = Turret.getInstance();

    @Override
    public void onStart() {
        // no-op
    }

    @Override
    public void onLoop() {
        if (turret.getForwardLimitSwitch()) {
            turret.reset(Rotation2d.fromDegrees(Constants.kMaxTurretAngle));
        } else if (turret.getReverseLimitSwitch()) {
            turret.reset(Rotation2d.fromDegrees(Constants.kMinTurretAngle));
        }
    }

    @Override
    public void onStop() {
        // no-op
    }

}
