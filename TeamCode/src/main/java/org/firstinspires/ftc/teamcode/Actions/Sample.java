package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HWMaps.Robot;

public class Sample extends Action {
    Robot robot;

    public Sample(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        robot.lift.getMotor().setTargetPosition(0);
    }

    @Override
    public void run() {
        robot.lift.getMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(0.2);
    }

    @Override
    public void kill() {
        actionIsComplete = true;
        robot.lift.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
