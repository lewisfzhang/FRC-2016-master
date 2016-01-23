package com.team254.logger;

import org.eclipse.paho.client.mqttv3.MqttException;

import java.util.HashMap;

/**
 * Interface for sending data to the Cheesy Logger dashboard
 */
public class CheesyLogger {

    public static enum CompetitionState {
        DISABLED("disabled"),
        TELEOP("teleop"),
        AUTO("auto");

        private final String mWireValue;

        CompetitionState(String wireValue) {
            mWireValue = wireValue;
        }
    }

    private final MqttSender mMqttSender;

    public static CheesyLogger makeCheesyLogger() {
        try {
            // TODO: unbreak mDNS and put a hostname here
            return new CheesyLogger(new MqttSender("tcp://localhost:1883"));
        } catch (MqttException e) {
            e.printStackTrace();
            throw new RuntimeException(e);
        }
    }

    private CheesyLogger(MqttSender mqttSender) {
        mMqttSender = mqttSender;
    }

    /**
     * Send a text log message.
     * @param message
     */
    public void sendLogMessage(String message) {
        HashMap<String, String> payload = makeEmptyLogPayload("log");
        payload.put("message", message);
        mMqttSender.sendPayload(payload, MqttSender.QOS.BEST_EFFORT);
    }

    /**
     * Send a value point which should be plotted against the current time in a time-series plot.
     * @param category
     * @param value
     */
    public void sendTimePlotPoint(String category, double value) {
        HashMap<String, String> payload = makeEmptyLogPayload("timeplot");
        payload.put("category", category);
        payload.put("value", Double.toString(value));
        mMqttSender.sendPayload(payload, MqttSender.QOS.BEST_EFFORT);
    }

    /**
     * Send an x/y pair to plot on a scatter-plot graph.
     * @param category
     * @param x
     * @param y
     */
    public void sendScatterPlotPoint(String category, double x, double y) {
        HashMap<String, String> payload = makeEmptyLogPayload("scatterplot");
        payload.put("category", category);
        payload.put("x", Double.toString(x));
        payload.put("y", Double.toString(y));
        mMqttSender.sendPayload(payload, MqttSender.QOS.BEST_EFFORT);
    }

    public void sendCompetitionState(CompetitionState competitionState) {
        HashMap<String, String> payload = makeEmptyLogPayload("competitionstate");
        payload.put("state", competitionState.mWireValue);
        mMqttSender.sendPayload(payload, MqttSender.QOS.AT_LEAST_ONCE);
    }

    private HashMap<String, String> makeEmptyLogPayload(String type) {
        HashMap<String, String> result = new HashMap<String, String>();
        result.put("walltime", Long.toString(System.currentTimeMillis()));
        result.put("type", type);
        return result;
    }


}
