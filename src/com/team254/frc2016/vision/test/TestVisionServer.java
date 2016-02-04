package com.team254.frc2016.vision.test;

import com.team254.frc2016.vision.TargetInfo;
import com.team254.frc2016.vision.VisionServer;
import com.team254.frc2016.vision.VisionUpdate;
import com.team254.frc2016.vision.VisionUpdateReceiver;

public class TestVisionServer {

    public static class TestReceiver implements VisionUpdateReceiver {

        @Override
        public void gotUpdate(VisionUpdate update) {
            System.out.println("Got update!");
            System.out.println("-- Time ago: " + update.getCapturedAgoMs());
            System.out.println("-- Time captured: " + update.getCapturedAtMs());
            System.out.println("-- Now: " + System.currentTimeMillis());
            System.out.println("-- Num targets: " + update.getTargets().size());
            for (int i = 0; i < update.getTargets().size(); i++) {
                TargetInfo target = update.getTargets().get(i);
                System.out.println("-- Target #" + i);
                System.out.println("---- Theta: " + target.getTheta());
                System.out.println("---- Distance: " + target.getDistance());
            }
            System.out.println("");
        }
    }

    public static void main(String args[]) {
        System.out.println("Starting");
        VisionServer server = VisionServer.getInstance();
        TestReceiver r = new TestReceiver();
        server.addVisionUpdateReceiver(r);
        System.out.println("Running");
        while (true) {
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }
}
