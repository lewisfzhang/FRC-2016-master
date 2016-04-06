package com.team254.frc2016.auto.actions;

import com.team254.frc2016.subsystems.Drive;
import com.team254.lib.util.Path;

public class FollowPathAction implements Action {

    private Drive mDrive = Drive.getInstance();

    private Path mPath;
    private boolean mReversed;

    public FollowPathAction(Path path, boolean reversed) {
        mPath = path;
        mReversed = reversed;
    }

    @Override
    public boolean isFinished() {
        boolean done = mDrive.isFinishedPath();
        if (done) {
            System.out.println("Finished path");
        }
        return done;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        mDrive.setVelocitySetpoint(0, 0);
    }

    @Override
    public void start() {
        mDrive.followPath(mPath, mReversed);
    }

}
