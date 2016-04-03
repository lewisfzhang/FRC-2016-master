package com.team254.frc2016.vision.test;

import com.team254.frc2016.vision.TargetInfo;
import com.team254.frc2016.vision.VisionServer;
import com.team254.frc2016.vision.VisionUpdate;
import com.team254.frc2016.vision.VisionUpdateReceiver;
import com.team254.frc2016.vision.messages.SetCameraModeMessage;
import com.team254.frc2016.vision.messages.VisionMessage;

public class TestVisionServer {

    public static class TestReceiver implements VisionUpdateReceiver {

        @Override
        public void gotUpdate(VisionUpdate update) {
            for (int i = 0; i < update.getTargets().size(); i++) {
                TargetInfo target = update.getTargets().get(i);
                System.out.println("" + target.getY() + " : " + target.getZ());
            }
        }
    }

    public static void main(String args[]) {
        System.out.println("Starting");
        VisionServer server = VisionServer.getInstance();
        TestReceiver r = new TestReceiver();
        server.addVisionUpdateReceiver(r);
        System.out.println("Running");
        int i = 0;
        while (true) {
            try {
                Thread.sleep(1000);
                VisionMessage message = (i % 20) < 10 ? SetCameraModeMessage.getVisionModeMessage() : SetCameraModeMessage.getIntakeModeMessage();
                server.sendMessage(message);
                System.out.println("Sending " + message.getMessage());
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            i++;
        }

    }
}
