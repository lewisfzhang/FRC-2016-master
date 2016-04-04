package com.team254.frc2016.auto;

import com.team254.frc2016.auto.actions.Action;
import com.team254.frc2016.loops.Loop;
import com.team254.frc2016.loops.Looper;

import java.util.concurrent.Semaphore;

public abstract class AutoModeBase {

    protected double m_update_rate = 1.0 / 50.0;
    protected boolean m_active = false;

    private Semaphore mWaitSemaphone = new Semaphore(0);

    protected abstract void routine() throws AutoModeEndedException;

    private Looper m_looper = new Looper();

    protected class ActionLoopCallback {
        void actionDone() {

        };
    }
    ActionLoopCallback mCallback = new ActionLoopCallback();


    private class ActionLoop implements Loop {

        private Action m_action;

        public ActionLoop(Action action) {
            m_action = action;
        }

        @Override
        public void onStart() {
            m_action.start();
        }

        @Override
        public void onLoop() {
            if (isActive() && !m_action.isFinished()) {
                m_action.update();
            } else {
                mWaitSemaphone.release();
            }
        }

        @Override
        public void onStop() {
            m_action.done();
        }
    }
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
        m_looper.clearAllLoops();
        m_looper.register(new ActionLoop(action));
        m_looper.start();
        try {
            mWaitSemaphone.acquire();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        m_looper.stop();
    }
}
