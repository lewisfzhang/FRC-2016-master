package com.team254.frc2016.vision;

import com.team254.frc2016.Constants;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Collections;

public class VisionServer implements Runnable {

    private static VisionServer s_instance = null;

    public static VisionServer getInstance() {
        if (s_instance == null) {
            s_instance = new VisionServer(Constants.kVisionUDPPort);
        }
        return s_instance;
    }

    private DatagramSocket socket;
    private boolean m_running = true;
    private ArrayList<VisionUpdateReceiver> receivers = new ArrayList<>();

    private VisionServer(int port) {
        try {
            socket = new DatagramSocket(port);
        } catch (SocketException e) {
            e.printStackTrace();
        }
        new Thread(this).start();
    }

    public void addVisionUpdateReceiver(VisionUpdateReceiver receiver) {
        if (!receivers.contains(receiver)) {
            receivers.add(receiver);
        }
    }

    public void removeVisionUpdateReceiver(VisionUpdateReceiver receiver) {
        if (receivers.contains(receiver)) {
            receivers.remove(receiver);
        }
    }

    @Override
    public void run() {
        byte[] buf = new byte[2048];
        DatagramPacket packet = new DatagramPacket(buf, buf.length);
        while (m_running) {
            try {
                socket.receive(packet);
                long timestamp = System.nanoTime();
                String message = new String(packet.getData(), 0, packet.getLength());
                VisionUpdate update = VisionUpdate.generateFromJsonString(timestamp, message);
                receivers.removeAll(Collections.singleton(null));
                if (update.isValid()) {
                    for (VisionUpdateReceiver receiver : receivers) {
                        receiver.gotUpdate(update);
                    }
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
