package com.team254.frc2016.auto.actions;

public interface Action {
    public abstract boolean isFinished();

    public abstract void update();

    public abstract void done();

    public abstract void start();
}
