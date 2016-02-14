package com.team254.logger;

/**
 * A java main method implementation for testing code by running it on your
 * laptop
 */
public class TestingMain {

    public static void main(String[] argv) {
        CheesyLogger cheesyLogger = CheesyLogger.makeCheesyLogger("localhost");
        cheesyLogger.sendLogMessage("hello");
        cheesyLogger.sendTimePlotPoint("my_plot", "pi", 3.14, 100);
        cheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.DISABLED);

        int modeChangeCount = 0;
        while (true) {
            cheesyLogger.sendTimePlotPoint("my_plot", "test_value", System.currentTimeMillis() % 10000, 100);
            cheesyLogger.sendTimePlotPoint("my_other_plot", "test_value2", System.currentTimeMillis() % 1000, 100);

            modeChangeCount++;
            if (modeChangeCount == 1000) {
                cheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.TELEOP);
            }
            if (modeChangeCount == 2000) {
                cheesyLogger.sendCompetitionState(CheesyLogger.CompetitionState.AUTO);
                modeChangeCount = 0;
            }

            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
