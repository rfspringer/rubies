package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HWMaps.Robot;

public class Lower extends Action {
    private Robot robot;
    private ElapsedTime timer = new ElapsedTime();

    public Lower(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        robot.lift.setTargetPosition(robot.lift.extendedLiftPosition());
    }

    @Override
    public void run() {
        timer.reset();
        while (!actionIsComplete) {
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.5);
            actionIsComplete = (robot.lift.getCurrentPosition() <= (robot.lift.extendedLiftPosition() - 10)) || timer.seconds() > 4;
        }
    }

    @Override
    public void kill() {
        robot.lift.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lift.setPower(0);
    }
}
