package org.firstinspires.ftc.teamcode.Actions;

import org.firstinspires.ftc.teamcode.HWMaps.Robot;

public abstract class Action {
    Robot robot = Robot.getInstance();
    boolean actionIsComplete = false;

    public abstract void init();

    public abstract void run();

    public void kill() {
        actionIsComplete = true;
    }
}
