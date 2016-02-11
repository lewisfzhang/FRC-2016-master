package com.team254.frc2016.vision;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import com.team254.lib.util.Rotation2d;

import java.util.ArrayList;
import java.util.List;

public class VisionUpdate {
    protected boolean valid = false;
    protected long capturedAgoMs;
    protected List<TargetInfo> targets;
    protected long capturedAtMs = 0;

    private static long getOptLong(Object n, long defaultValue) {
        if (n == null) {
            return defaultValue;
        }
        return (long) n;
    }

    private static JSONParser parser = new JSONParser();

    // Example json string
    // { "capturedAgoMs" : 100, "targets": [{"angle": 5.4, "distance": 5.5}] }
    public static VisionUpdate generateFromJsonString(long timestampNanos, String updateString) {
        long startMs = timestampNanos / 1000000L;
        VisionUpdate update = new VisionUpdate();
        try {
            JSONObject j = (JSONObject) parser.parse(updateString);
            long capturedAgoMs = getOptLong(j.get("capturedAgoMs"), 0);
            if (capturedAgoMs == 0) {
                update.valid = false;
                return update;
            }
            update.capturedAgoMs = capturedAgoMs;
            update.capturedAtMs = startMs - capturedAgoMs;
            JSONArray targets = (JSONArray) j.get("targets");
            ArrayList<TargetInfo> targetInfos = new ArrayList<>(targets.size());
            for (Object targetObj : targets) {
                JSONObject target = (JSONObject) targetObj;
                double distance = (double) target.get("distance");
                Rotation2d angle = Rotation2d.fromDegrees((double) target.get("angle"));
                targetInfos.add(new TargetInfo(distance, angle));
            }
            update.targets = targetInfos;
            update.valid = true;
        } catch (ParseException e) {
            System.err.println("Parse error: " + e);
            e.printStackTrace();
        } catch (ClassCastException e) {
            System.err.println("Data type error: " + e);
            e.printStackTrace();
        }
        return update;
    }

    public List<TargetInfo> getTargets() {
        return targets;
    }

    public boolean isValid() {
        return valid;
    }

    public long getCapturedAgoMs() {
        return capturedAgoMs;
    }

    public long getCapturedAtMs() {
        return capturedAtMs;
    }

}
