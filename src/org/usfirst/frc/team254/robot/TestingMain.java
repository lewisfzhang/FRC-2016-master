package org.usfirst.frc.team254.robot;

/**
 * A java main method implementation for testing code by running it on your laptop
 */
public class TestingMain {

    public static void main(String[] argv) {
        CheesyLogger cheesyLogger = CheesyLogger.makeCheesyLogger();
        cheesyLogger.sendLogMessage("hello");
        cheesyLogger.sendTimePlotPoint("pi", 3.14);
        cheesyLogger.sendScatterPlotPoint("position", 23.2323232323, 42);

        while (true) {
            cheesyLogger.sendTimePlotPoint("test_value", System.currentTimeMillis() % 10000);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
