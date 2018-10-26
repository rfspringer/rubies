package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HWMaps.Robot;

public class Hang extends Action {
    private Robot robot = Robot.getInstance();

    @Override
    public void init() {
        robot.lift.getMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void run() {
        if (!actionIsComplete) {
            robot.lift.getMotor().setTargetPosition(robot.lift.extendedLiftPosition());
        }
    }
}
