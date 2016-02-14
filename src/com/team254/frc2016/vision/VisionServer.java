package com.team254.frc2016.vision;

import com.team254.frc2016.Constants;

import java.io.IOException;
import java.io.InputStream;
import java.net.*;
import java.util.ArrayList;
import java.util.Collections;

public class VisionServer implements Runnable {

    private static VisionServer s_instance = null;
    private ServerSocket m_server_socket;
    private boolean m_running = true;
    private int m_port;
    private ArrayList<VisionUpdateReceiver> receivers = new ArrayList<>();
    AdbBridge adb = new AdbBridge();

    public static VisionServer getInstance() {
        if (s_instance == null) {
            s_instance = new VisionServer(Constants.kAndroidAppTcpPort);
        }
        return s_instance;
    }

    protected class ServerThread implements Runnable
    {
        private Socket m_socket;
        public ServerThread(Socket socket) {
            m_socket = socket;
        }

        @Override
        public void run() {
            if (m_socket == null) {
                return;
            }
            try {
                InputStream is = m_socket.getInputStream();
                byte[] buffer = new byte[2048];
                int read;
                while (m_socket.isConnected() && (read = is.read(buffer)) != -1) {
                    long timestamp = System.nanoTime();
                    String message = new String(buffer, 0, read);
                    if ("PING".equals(message)) {
                        m_socket.getOutputStream().write("PONG".getBytes());
                        continue;
                    }
                    VisionUpdate update = VisionUpdate.generateFromJsonString(timestamp, message);
                    receivers.removeAll(Collections.singleton(null));
                    if (update.isValid()) {
                        for (VisionUpdateReceiver receiver : receivers) {
                            receiver.gotUpdate(update);
                        }
                    }
                }
            } catch(IOException e) {
                System.err.println("Could not talk to socket");
            }
            if (m_socket != null) {
                try {
                    m_socket.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private VisionServer(int port) {
        try {
            adb = new AdbBridge();
            m_port = port;
            m_server_socket = new ServerSocket(port);
            adb.start();
            adb.reversePortForward(port, port);
        } catch (IOException e) {
            e.printStackTrace();
        }
        new Thread(this).start();
    }

    public void restartAdb() {
        adb.restart();
        adb.reversePortForward(m_port, m_port);
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
        while (m_running) {
            try {
                Socket p = m_server_socket.accept();
                new Thread(new ServerThread(p)).start();
            } catch (IOException e) {
                System.err.println("Issue accepting socket connection!");
            }
        }
    }
}
