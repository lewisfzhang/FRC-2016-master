package com.team254.frc2016.loops;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Notifier;

public class Looper {
    public final double kPeriod = 0.01; // 100Hz

    Notifier notifier_;
    boolean running_;
    List<Loop> loops_;

    Runnable runnable_ = new Runnable() {
        @Override
        public void run() {
            synchronized (Looper.this) {
                if (running_) {
                    for (Loop loop : loops_) {
                        loop.onLoop();
                    }
                }
            }
        }
    };

    public Looper() {
        notifier_ = new Notifier(runnable_);
        running_ = false;
        loops_ = new ArrayList<>();
    }

    public synchronized void register(Loop loop) {
        loops_.add(loop);
    }

    public synchronized void start() {
        if (!running_) {
            System.out.println("Starting loops");
            for (Loop loop : loops_) {
                loop.onStart();
            }
            running_ = true;
            notifier_.startPeriodic(kPeriod);
        }
    }

    public synchronized void stop() {
        if (running_) {
            System.out.println("Stopping loops");
            running_ = false;
            notifier_.stop();
            for (Loop loop : loops_) {
                System.out.println("Stopping " + loop);
                loop.onStop();
            }
        }
    }
}
