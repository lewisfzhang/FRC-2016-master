package com.team254.logger;

import org.eclipse.paho.client.mqttv3.MqttException;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * Interface for sending data to the Cheesy Logger dashboard
 */
public class CheesyLogger {

    public enum CompetitionState {
        DISABLED("disabled"), TELEOP("teleop"), AUTO("auto");

        private final String mWireValue;

        CompetitionState(String wireValue) {
            mWireValue = wireValue;
        }
    }

    private final MqttSender mMqttSender;
    private final HashMap<ArrayList<String>, Long> mLastSampleMap;

    public static CheesyLogger makeCheesyLogger(String mqttServerHostname) {
        try {
            return new CheesyLogger(new MqttSender("tcp://" + mqttServerHostname + ":1883"));
        } catch (MqttException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }

    private CheesyLogger(MqttSender mqttSender) {
        mMqttSender = mqttSender;
        mLastSampleMap = new HashMap<>();
    }

    /**
     * Send a text log message.
     * 
     * @param message
     */
    public void sendLogMessage(String message) {
        HashMap<String, String> payload = makeEmptyLogPayload("log", System.currentTimeMillis());
        payload.put("message", message);
        mMqttSender.sendPayload(payload, MqttSender.QOS.BEST_EFFORT);
    }

    /**
     * Send a value point which should be plotted against the current time in a
     * time-series plot.
     * 
     * @param category
     *            Which plot this data should go on
     * @param field
     *            Which data in this line this point is assocaited with (aka,
     *            which line it belongs to)
     * @param value
     *            The poisition of the point on the plot
     * @param minSampleMillis
     *            The min milliseconds between 2 points to send for this
     *            category
     */
    public void sendTimePlotPoint(String category, String field, double value, long minSampleMillis) {
        // Apply Sampling logic first
        long walltime = System.currentTimeMillis();
        ArrayList<String> sampleKey = new ArrayList<>(3);
        String type = "timeplot";
        sampleKey.add(type);
        sampleKey.add(category);
        sampleKey.add(field);
        long lastSampleTime = mLastSampleMap.getOrDefault(sampleKey, 0L);
        if (walltime - lastSampleTime < minSampleMillis) {
            return;
        }
        mLastSampleMap.put(sampleKey, walltime);

        // Send the message
        HashMap<String, String> payload = makeEmptyLogPayload(type, walltime);
        payload.put("category", category);
        payload.put("field", field);
        payload.put("value", Double.toString(value));
        mMqttSender.sendPayload(payload, MqttSender.QOS.BEST_EFFORT);
    }

    public void sendCompetitionState(CompetitionState competitionState) {
        HashMap<String, String> payload = makeEmptyLogPayload("competitionstate", System.currentTimeMillis());
        payload.put("state", competitionState.mWireValue);
        mMqttSender.sendPayload(payload, MqttSender.QOS.AT_LEAST_ONCE);
    }

    private HashMap<String, String> makeEmptyLogPayload(String type, long walltime) {
        HashMap<String, String> result = new HashMap<String, String>();
        result.put("walltime", Long.toString(walltime));
        result.put("type", type);
        return result;
    }
}
