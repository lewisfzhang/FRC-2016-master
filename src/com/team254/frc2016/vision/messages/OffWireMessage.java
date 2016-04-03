package com.team254.frc2016.vision.messages;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class OffWireMessage extends VisionMessage {

    private boolean mValid = false;
    private String mType = "unknown";
    private String mMessage = "{}";

    public OffWireMessage(String message) {
        JSONParser parser = new JSONParser();
        try {
            JSONObject j = (JSONObject) parser.parse(message);
            mType = (String) j.get("type");
            mMessage = (String) j.get("message");
            mValid = true;
        } catch (ParseException e) {
        }
    }

    public boolean isValid() {
        return mValid;
    }

    @Override
    public String getType() {
        return mType;
    }

    @Override
    public String getMessage() {
        return mMessage;
    }
}
