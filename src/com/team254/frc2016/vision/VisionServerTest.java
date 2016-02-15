package com.team254.frc2016.vision;

public class VisionServerTest {
    public static class TestReceiver implements VisionUpdateReceiver {
        @Override
        public void gotUpdate(VisionUpdate update) {
            System.out.println("num targets: " + update.getTargets().size());
            for (int i = 0; i < update.getTargets().size(); i++) {
                TargetInfo target = update.getTargets().get(i);
                System.out.println("Target: " + target.getAngle() + ", " + target.getDistance());
            }
        }
    }

    public static void main(String[] args) {
        VisionServer visionServer = VisionServer.getInstance();
        visionServer.addVisionUpdateReceiver(new TestReceiver());
        while (true) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
