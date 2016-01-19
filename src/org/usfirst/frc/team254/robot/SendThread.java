package org.usfirst.frc.team254.robot;

import com.sun.istack.internal.Nullable;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;

public class SendThread implements Runnable {

    /** Limits the size of a single MQTT message */
    private static final int MAX_MESSAGES_PER_MQTT_MESSAGE = 10;

    private final ConcurrentLinkedQueue<Map<String, String>> mMessageQueue;
    private final Thread mThread;

    public SendThread() {
        mMessageQueue = new ConcurrentLinkedQueue<>();
        mThread = new Thread(this);
        mThread.start();
    }


    @Override
    public void run() {
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

    private void drainQueue() {
        StringBuilder builder = new StringBuilder("[");
        boolean firstElement = true;
        for (int i = 0; i < MAX_MESSAGES_PER_MQTT_MESSAGE; ++i) {
            @Nullable Map<String, String> nextMessage = mMessageQueue.poll();
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

        // TODO: send to MQTT
        System.out.println(builder.toString());
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

    public void sendPayload(Map<String, String> payload) {
        mMessageQueue.add(payload);
    }

}
