package com.team254.frc2016.vision.messages;


import org.json.simple.JSONObject;

public abstract class VisionMessage {

    public abstract String getType();

    public abstract String getMessage();

    public String toJson() {
        JSONObject j = new JSONObject();
        j.put("type", getType());
        j.put("message", getMessage());
        return j.toString();
    }

}
