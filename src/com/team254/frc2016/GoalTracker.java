package com.team254.frc2016;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;

import com.team254.lib.util.Translation2d;

public class GoalTracker {
    // Track reports contain all of the relevant information about a given goal
    // track.
    public static class TrackReport {
        // Translation from the odometric frame to the goal
        public Translation2d odometric_to_goal;

        // The timestamp of the latest time that the goal has been observed
        public double latest_timestamp;

        // The percentage of the goal tracking time during which this goal has
        // been observed (0 to 1)
        public double stability;

        // The track id
        public int id;

        public TrackReport(GoalTrack track) {
            this.odometric_to_goal = track.getSmoothedPosition();
            this.latest_timestamp = track.getLatestTimestamp();
            this.stability = track.getStability();
            this.id = track.getId();
        }
    }

    // TrackReportComparators are used in the case that multiple tracks are
    // active (e.g. we see or have recently seen multiple goals).
    // They contain heuristics used to pick which track we should aim at by
    // calculating a score for each track (highest score wins).
    public static class TrackReportComparator implements Comparator<TrackReport> {
        // Reward tracks for being more stable (seen in more frames)
        double mStabilityWeight;
        // Reward tracks for being recently observed
        double mAgeWeight;
        double mCurrentTimestamp;
        // Reward tracks for being continuations of tracks that we are already
        // tracking
        double mSwitchingWeight;
        int mLastTrackId;

        public TrackReportComparator(double stability_weight, double age_weight, double switching_weight,
                int last_track_id, double current_timestamp) {
            this.mStabilityWeight = stability_weight;
            this.mAgeWeight = age_weight;
            this.mSwitchingWeight = switching_weight;
            this.mLastTrackId = last_track_id;
            this.mCurrentTimestamp = current_timestamp;
        }

        double score(TrackReport report) {
            double stability_score = mStabilityWeight * report.stability;
            double age_score = mAgeWeight
                    * Math.max(0, (Constants.kMaxGoalTrackAge - (mCurrentTimestamp - report.latest_timestamp))
                            / Constants.kMaxGoalTrackAge);
            double switching_score = (report.id == mLastTrackId ? mSwitchingWeight : 0);
            return stability_score + age_score + switching_score;
        }

        @Override
        public int compare(TrackReport o1, TrackReport o2) {
            double diff = score(o1) - score(o2);
            // <0 if o1 is better than o2
            if (diff < 0) {
                return 1;
            } else if (diff > 0) {
                return -1;
            } else {
                return 0;
            }
        }
    }

    List<GoalTrack> mCurrentTracks = new ArrayList<>();
    int mNextId = 0;

    public GoalTracker() {
    }

    public void reset() {
        mCurrentTracks.clear();
    }

    public void update(double timestamp, List<Translation2d> odometric_to_goals) {
        // System.out.println("START GoalTracker::Update");
        // System.out.println("Current number of tracks: " +
        // mCurrentTracks.size());
        boolean hasUpdatedTrack = false;
        // Try to update existing tracks
        for (Translation2d target : odometric_to_goals) {
            // System.out.println("Trying target " + target.toString());
            for (GoalTrack track : mCurrentTracks) {
                // System.out.println("Trying track " + track.getId());
                if (!hasUpdatedTrack) {
                    if (track.tryUpdate(timestamp, target)) {
                        hasUpdatedTrack = true;
                        // System.out.println("UPDATED TRACK");
                    } else {
                        // System.out.println("COULD NOT UPDATE TRACK");
                    }
                } else {
                    track.emptyUpdate();
                    // System.out.println("SKIPPING TRACK");
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
        // If all tracks are dead, start new tracks for any detections
        if (mCurrentTracks.isEmpty()) {
            for (Translation2d target : odometric_to_goals) {
                // System.out.println("STARTING NEW TRACK");
                mCurrentTracks.add(GoalTrack.makeNewTrack(timestamp, target, mNextId));
                ++mNextId;
            }
        }
        // System.out.println("End GoalTracker::Update");
    }

    public boolean hasTracks() {
        return !mCurrentTracks.isEmpty();
    }

    public List<TrackReport> getTracks() {
        List<TrackReport> rv = new ArrayList<>();
        for (GoalTrack track : mCurrentTracks) {
            rv.add(new TrackReport(track));
        }
        return rv;
    }
}
