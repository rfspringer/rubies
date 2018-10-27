package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HWMaps.Robot;

public class Hang extends Action {
    Robot robot;

    public Hang(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        robot.lift.getMotor().setTargetPosition(0);
    }

    @Override
    public void run() {
        if (!actionIsComplete) {
            robot.lift.getMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.2);
        }
    }
}
