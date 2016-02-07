package com.team254.logger;

import org.eclipse.paho.client.mqttv3.*;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import java.util.Map;
import java.util.UUID;
import java.util.concurrent.ConcurrentLinkedQueue;

public class MqttSender implements Runnable, MqttCallback {

    public enum QOS {
        BEST_EFFORT(0), AT_LEAST_ONCE(1), EXACTLY_ONCE(2);

        private final int mQosValue;

        QOS(int qosValue) {
            mQosValue = qosValue;
        }
    }

    /** Limits the size of a single MQTT message */
    private static final int MAX_LOGS_PER_MESSAGE = 50;

    /** Limits the number of messages which will be enqeued in memory */
    private static final int MAX_MESSAGES_QUEUE_LENGTH = 200;

    private final ConcurrentLinkedQueue<LogElement> mLogQueue;
    private final Thread mThread;

    /** Guarded by "this" */
    private final MqttClient mMqttClient;

    public MqttSender(String mqttBrokerServer) throws MqttException {
        mLogQueue = new ConcurrentLinkedQueue<>();
        synchronized (this) {
            mMqttClient = new MqttClient(mqttBrokerServer, "robot_" + UUID.randomUUID(), new MemoryPersistence());
            mMqttClient.setCallback(this);
        }

        mThread = new Thread(this);
        mThread.start();
    }

    public void sendPayload(Map<String, String> payload, QOS qos) {
        if (mLogQueue.size() < MAX_MESSAGES_QUEUE_LENGTH) {
            mLogQueue.add(new LogElement(payload, qos));
        } else {
            System.out.println("CheesyLogger Message Queue full");
        }
    }

    @Override
    public void connectionLost(Throwable throwable) {
        throwable.printStackTrace();
        forceMqttConnect();
    }

    @Override
    public void messageArrived(String s, MqttMessage mqttMessage) throws Exception {
    }

    @Override
    public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken) {
    }

    @Override
    public void run() {
        forceMqttConnect();

        while (true) {
            maybeSendMessage();
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    private void forceMqttConnect() {
        while (true) {
            try {
                synchronized (this) {
                    if (mMqttClient.isConnected()) {
                        return;
                    }
                    mMqttClient.connect();
                }
                return;
            } catch (MqttException e) {
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e1) {
                    e1.printStackTrace();
                }
            }
        }
    }

    private void maybeSendMessage() {
        LogElement firstElement = mLogQueue.peek();
        if (firstElement == null) {
            return;
        }
        // A single MQTT message must all have the same QOS
        QOS curQos = firstElement.mQOS;
        JSONArray jsonMessages = new JSONArray();
        int numLogs = 0;
        while (numLogs < MAX_LOGS_PER_MESSAGE) {
            LogElement nextObject = mLogQueue.peek();
            if (nextObject == null || nextObject.mQOS != curQos) {
                break;
            }
            mLogQueue.remove();
            jsonMessages.add(new JSONObject(nextObject.mPayload));
            numLogs++;
        }

        try {
            String payload = jsonMessages.toJSONString();
            synchronized (this) {
                mMqttClient.publish("/robot_logging", payload.getBytes(), curQos.mQosValue, false);
            }
        } catch (MqttException e) {
            e.printStackTrace();
        }
    }

    private static class LogElement {
        private final Map<String, String> mPayload;
        private final QOS mQOS;

        private LogElement(Map<String, String> payload, QOS qos) {
            mPayload = payload;
            mQOS = qos;
        }
    }
}
