package com.team254.frc2016.auto;

import com.team254.frc2016.auto.actions.Action;

public abstract class AutoModeBase {
    protected double m_update_rate = 1.0 / 50.0;
    protected boolean m_active = false;

    protected abstract void routine() throws AutoModeEndedException;

    public void run() {
        m_active = true;
        try {
            routine();
        } catch (AutoModeEndedException e) {
            System.out.println("Auto mode done, ended early");
            return;
        }
        System.out.println("Auto mode done");
    }

    public void stop() {
        m_active = false;
    }

    public boolean isActive() {
        return m_active;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if (!isActive()) {
            throw new AutoModeEndedException();
        }
        return isActive();
    }

    public void runAction(Action action) throws AutoModeEndedException {
        isActiveWithThrow();
        action.start();
        while (isActiveWithThrow() && !action.isFinished()) {
            action.update();
            long waitTime = (long) (m_update_rate * 1000.0);
            try {

                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        action.done();
    }

}
