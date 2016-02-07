package com.team254.logger;

import org.eclipse.paho.client.mqttv3.MqttException;

import java.util.HashMap;

/**
 * Interface for sending data to the Cheesy Logger dashboard
 */
public class CheesyLogger {

    public static enum CompetitionState {
        DISABLED("disabled"), TELEOP("teleop"), AUTO("auto");

        private final String mWireValue;

        CompetitionState(String wireValue) {
            mWireValue = wireValue;
        }
    }

    private final MqttSender mMqttSender;

    public static CheesyLogger makeCheesyLogger() {
        try {
            // TODO: unbreak mDNS and put a hostname here
            return new CheesyLogger(new MqttSender("tcp://10.2.54.195:1883"));
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
     * 
     * @param message
     */
    public void sendLogMessage(String message) {
        HashMap<String, String> payload = makeEmptyLogPayload("log");
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
     */
    public void sendTimePlotPoint(String category, String field, double value) {
        HashMap<String, String> payload = makeEmptyLogPayload("timeplot");
        payload.put("category", category);
        payload.put("field", field);
        payload.put("value", Double.toString(value));
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
