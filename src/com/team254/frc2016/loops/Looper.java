package com.team254.frc2016.loops;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Notifier;

public class Looper {
    public final double kPeriod = 0.01; // 100Hz

    private boolean running_;

    private final Notifier notifier_;
    private final List<Loop> loops_;
    private final Object taskRunningLock_ = new Object();
    private final Runnable runnable_ = new Runnable() {
        @Override
        public void run() {
            synchronized (taskRunningLock_) {
                if (running_) {
                    // double now = Timer.getFPGATimestamp();
                    for (Loop loop : loops_) {
                        loop.onLoop();
                    }
                    // System.out.println("Looper took " +
                    // (Timer.getFPGATimestamp() - now) + " seconds");
                }
            }
        }
    };

    public Looper() {
        notifier_ = new Notifier(runnable_);
        running_ = false;
        loops_ = new ArrayList<>();
    }

    public synchronized void clearAllLoops() {
        synchronized (taskRunningLock_) {
            loops_.clear();
        }
    }

    public synchronized void register(Loop loop) {
        synchronized (taskRunningLock_) {
            loops_.add(loop);
        }
    }

    public synchronized void start() {
        if (!running_) {
            System.out.println("Starting loops");
            synchronized (taskRunningLock_) {
                for (Loop loop : loops_) {
                    loop.onStart();
                }
                running_ = true;
            }
            notifier_.startPeriodic(kPeriod);
        }
    }

    public synchronized void stop() {
        if (running_) {
            System.out.println("Stopping loops");
            notifier_.stop();
            synchronized (taskRunningLock_) {
                running_ = false;
                for (Loop loop : loops_) {
                    System.out.println("Stopping " + loop);
                    loop.onStop();
                }
            }
        }
    }
}
