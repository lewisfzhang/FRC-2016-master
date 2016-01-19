package org.usfirst.frc.team254.robot;

import java.util.HashMap;

/**
 * Interface for sending data to the Cheesy Logger dashboard
 */
public class CheesyLogger {

    private final SendThread mSendThread;

    public static CheesyLogger makeCheesyLogger() {
        return new CheesyLogger(new SendThread());
    }

    private CheesyLogger(SendThread sendThread) {
        mSendThread = sendThread;
    }

    /**
     * Send a text log message.
     * @param message
     */
    public void sendLogMessage(String message) {
        HashMap<String, String> payload = makeEmptyLogPayload("log");
        payload.put("message", message);
        mSendThread.sendPayload(payload);
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
        mSendThread.sendPayload(payload);
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
        mSendThread.sendPayload(payload);
    }

    private HashMap<String, String> makeEmptyLogPayload(String type) {
        HashMap<String, String> result = new HashMap<String, String>();
        result.put("walltime", Long.toString(System.currentTimeMillis()));
        result.put("type", type);
        return result;
    }


}
