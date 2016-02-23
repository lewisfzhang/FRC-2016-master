package com.team254.frc2016;

import java.util.Iterator;
import java.util.Map;
import java.util.TreeMap;

import com.team254.lib.util.Translation2d;

import edu.wpi.first.wpilibj.Timer;

public class GoalTrack {
    public static final double kMaxAge = 0.5;
    public static final double kMaxTrackerDistance = 12.0;
    Map<Double, Translation2d> mObservedPositions = new TreeMap<>();
    Translation2d mSmoothedPosition = new Translation2d();

    GoalTrack() {
    }

    public static GoalTrack makeNewTrack(double timestamp, Translation2d first_observation) {
        GoalTrack rv = new GoalTrack();
        rv.mObservedPositions.put(timestamp, first_observation);
        rv.mSmoothedPosition = first_observation;
        return rv;
    }

    public void emptyUpdate() {
        pruneByTime();
    }

    // Returns true if the track was updated
    public boolean tryUpdate(double timestamp, Translation2d new_observation) {
        double distance = mSmoothedPosition.inverse().translateBy(new_observation).norm();
        if (distance < kMaxTrackerDistance) {
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
        double delete_before = Timer.getFPGATimestamp() - kMaxAge;
        for (Iterator<Map.Entry<Double, Translation2d>> it = mObservedPositions.entrySet().iterator(); it.hasNext();) {
            Map.Entry<Double, Translation2d> entry = it.next();
            if (entry.getKey() < delete_before) {
                it.remove();
            }
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
}
