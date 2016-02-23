package com.team254.frc2016;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import com.team254.lib.util.Translation2d;

public class GoalTracker {
    List<GoalTrack> mCurrentTracks = new ArrayList<>();
    GoalTrack mLatestTrack = null;

    public GoalTracker() {
    }

    public void reset() {
        mCurrentTracks.clear();
    }

    public void update(double timestamp, List<Translation2d> odometric_to_goals) {
        // System.out.println("START GoalTracker::Update");
        boolean hasUpdatedTrack = false;
        // Try to update existing tracks
        for (Translation2d target : odometric_to_goals) {
            if (mLatestTrack != null && mLatestTrack.isAlive() && mLatestTrack.tryUpdate(timestamp, target)) {
                hasUpdatedTrack = true;
                // TODO improve logic when two targets are present
                // System.out.println("UPDATED LATEST TRACK");
            }
            for (GoalTrack track : mCurrentTracks) {
                if (!hasUpdatedTrack) {
                    if (track.tryUpdate(timestamp, target)) {
                        hasUpdatedTrack = true;
                        mLatestTrack = track;
                        // System.out.println("UPDATED NON-LATEST TRACK");
                    }
                } else {
                    track.emptyUpdate();
                }
            }
        }
        // Prune any tracks that have died
        for (Iterator<GoalTrack> it = mCurrentTracks.iterator(); it.hasNext();) {
            GoalTrack track = it.next();
            if (!track.isAlive()) {
                it.remove();
                // System.out.println("KILLED OLD TRACK");
            }
        }
        // If all tracks are dead, clear the latest track
        if (mCurrentTracks.isEmpty()) {
            // System.out.println("ALL TRACKS ARE DEAD");
            mLatestTrack = null;
        }
        // If all tracks are dead, start new tracks for any detections
        if (mCurrentTracks.isEmpty()) {
            for (Translation2d target : odometric_to_goals) {
                // System.out.println("STARTING NEW TRACK");
                mLatestTrack = GoalTrack.makeNewTrack(timestamp, target);
                mCurrentTracks.add(mLatestTrack);
            }
        }
        // System.out.println("End GoalTracker::Update");
    }

    public boolean hasTracks() {
        return !mCurrentTracks.isEmpty();
    }

    public Translation2d getLatestSmoothedTrack() {
        if (mLatestTrack == null || !mLatestTrack.isAlive()) {
            return null;
        }
        return mLatestTrack.getSmoothedPosition();
    }
}
