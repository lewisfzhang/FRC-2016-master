package com.team254.frc2016.auto.actions;

import com.team254.frc2016.subsystems.Drive;

public class WaitForPathMarkerAction implements Action {

    private Drive mDrive = Drive.getInstance();
    private String mMarker;

    public WaitForPathMarkerAction(String marker) {
        mMarker = marker;
    }

    @Override
    public boolean isFinished() {
        return mDrive.getPathMarkersCrossed().contains(mMarker);
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
    }

    @Override
    public void start() {
    }

}
