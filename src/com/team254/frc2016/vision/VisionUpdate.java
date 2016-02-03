package com.team254.frc2016.vision;


import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.util.ArrayList;
import java.util.List;

public class VisionUpdate {
    protected boolean valid = false;
    protected long capturedAgoMs;
    protected List<TargetInfo> targets;

    private static JSONParser parser  = new JSONParser();

    // Example json string
    // { "capturedAgoMs" : 100, "targets": [{"theta": 5.4, "distance": 5.5}] }
    public static VisionUpdate generateFromJsonString(String updateString) {
        VisionUpdate update = new VisionUpdate();
        try {
            JSONObject j = (JSONObject) parser.parse(updateString);
            long capturedAgoMs = (long) j.get("capturedAgoMs");
            update.capturedAgoMs = capturedAgoMs;
            JSONArray targets = (JSONArray) j.get("targets");
            ArrayList<TargetInfo> targetInfos = new ArrayList<>(targets.size());
            for (Object targetObj : targets) {
                JSONObject target = (JSONObject) targetObj;
                double distance = (double) target.get("distance");
                double theta = (double) target.get("theta");
                targetInfos.add(new TargetInfo(theta, distance));
            }
            update.targets = targetInfos;
            update.valid = true;
        } catch (ParseException e) {
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


}
