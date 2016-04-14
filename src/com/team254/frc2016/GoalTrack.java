package com.team254.frc2016;

import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;

import com.team254.lib.util.Translation2d;

import edu.wpi.first.wpilibj.Timer;

public class GoalTrack {
    Map<Double, Translation2d> mObservedPositions = new TreeMap<>();
    Translation2d mSmoothedPosition = null;
    int mId;

    private GoalTrack() {
    }

    public static GoalTrack makeNewTrack(double timestamp, Translation2d first_observation, int id) {
        GoalTrack rv = new GoalTrack();
        rv.mObservedPositions.put(timestamp, first_observation);
        rv.mSmoothedPosition = first_observation;
        // System.out.println("Set smoothed position to " +
        // first_observation.toString());
        rv.mId = id;
        return rv;
    }

    public void emptyUpdate() {
        pruneByTime();
    }

    // Returns true if the track was updated
    public boolean tryUpdate(double timestamp, Translation2d new_observation) {
        if (!isAlive()) {
            return false;
        }
        double distance = mSmoothedPosition.inverse().translateBy(new_observation).norm();
        if (distance < Constants.kMaxTrackerDistance) {
            mObservedPositions.put(timestamp, new_observation);
            pruneByTime();
            return true;
        } else {
            emptyUpdate();
            return false;
        }
    }

    public boolean isAlive() {
        return mObservedPositions.size() > 0;
    }

    void pruneByTime() {
        double delete_before = Timer.getFPGATimestamp() - Constants.kMaxGoalTrackAge;
        for (Iterator<Map.Entry<Double, Translation2d>> it = mObservedPositions.entrySet().iterator(); it.hasNext();) {
            Map.Entry<Double, Translation2d> entry = it.next();
            if (entry.getKey() < delete_before) {
                it.remove();
            }
        }
        if (mObservedPositions.isEmpty()) {
            mSmoothedPosition = null;
        } else {
            smooth();
        }
    }

    void smooth() {
        if (isAlive()) {
            double x = 0;
            double y = 0;
            for (Map.Entry<Double, Translation2d> entry : mObservedPositions.entrySet()) {
                x += entry.getValue().getX();
                y += entry.getValue().getY();
            }
            x /= mObservedPositions.size();
            y /= mObservedPositions.size();
            mSmoothedPosition = new Translation2d(x, y);
        }
    }

    public Translation2d getSmoothedPosition() {
        return mSmoothedPosition;
    }

    public double getLatestTimestamp() {
        return mObservedPositions.keySet().stream().max(Double::compareTo).orElse(0.0);
    }

    public double getStability() {
        return Math.min(1.0, mObservedPositions.size() / (Constants.kCameraFrameRate * Constants.kMaxGoalTrackAge));
    }

    public int getId() {
        return mId;
    }
}
