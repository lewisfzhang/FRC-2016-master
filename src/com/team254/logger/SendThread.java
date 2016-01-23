package com.team254.logger;

import org.eclipse.paho.client.mqttv3.*;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;

import java.util.Map;
import java.util.UUID;
import java.util.concurrent.ConcurrentLinkedQueue;

public class SendThread implements Runnable, MqttCallback {

    /** Limits the size of a single MQTT message */
    private static final int MAX_MESSAGES_PER_MQTT_MESSAGE = 10;

    /** Limits the number of messages which will be enqeued in memory */
    private static final int MAX_MESSAGES_QUEUE_LENGTH = 200;

    private final ConcurrentLinkedQueue<Map<String, String>> mMessageQueue;
    private final Thread mThread;

    /** Guarded by "this" */
    private final MqttClient mMqttClient;

    public SendThread(String mqttBrokerServer) throws MqttException {
        mMessageQueue = new ConcurrentLinkedQueue<>();
        synchronized (this) {
            mMqttClient = new MqttClient(
                    mqttBrokerServer,
                    "robot_" + UUID.randomUUID(),
                    new MemoryPersistence());
            mMqttClient.setCallback(this);
        }

        mThread = new Thread(this);
        mThread.start();
    }

    public void sendPayload(Map<String, String> payload) {
        if (mMessageQueue.size() < MAX_MESSAGES_QUEUE_LENGTH) {
            mMessageQueue.add(payload);
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
    public void messageArrived(String s, MqttMessage mqttMessage) throws Exception {}

    @Override
    public void deliveryComplete(IMqttDeliveryToken iMqttDeliveryToken) {}

    @Override
    public void run() {
        forceMqttConnect();

        while (true) {
            if (!mMessageQueue.isEmpty()) {
                drainQueue();
            }

            try {
                Thread.sleep(50);
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

    private void drainQueue() {
        StringBuilder builder = new StringBuilder("[");
        boolean firstElement = true;
        for (int i = 0; i < MAX_MESSAGES_PER_MQTT_MESSAGE; ++i) {
            Map<String, String> nextMessage = mMessageQueue.poll();
            if (nextMessage == null) {
                break;
            }
            if (firstElement) {
                firstElement = false;
            } else {
                builder.append(",");
            }
            appendMessage(builder, nextMessage);
        }
        builder.append("]");

        try {
            String payload = builder.toString();
            System.out.println(payload);
            synchronized (this) {
                mMqttClient.publish("/robot_logging", payload.getBytes(), 0, false);
            }
        } catch (MqttException e) {
            e.printStackTrace();
        }
    }

    private void appendMessage(StringBuilder builder, Map<String, String> message) {
        builder.append("{");
        boolean firstField = true;
        for (Map.Entry<String, String> field : message.entrySet()) {
            if (firstField) {
                firstField = false;
            } else {
                builder.append(",");
            }
            builder.append("\"")
                    .append(field.getKey())
                    .append("\":\"")
                    .append(field.getValue())
                    .append("\"");

        }
        builder.append("}");
    }


}
