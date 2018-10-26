package org.firstinspires.ftc.teamcode.Actions;

public abstract class Action {
    boolean actionIsComplete = false;

    public abstract void init();

    public abstract void run();

    public void kill() {
        actionIsComplete = true;
    }
}
