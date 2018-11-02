package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HWMaps.Robot;

public class Hang {
    boolean actionIsComplete;
//
//    public Hang() {
//        Robot robot = Robot.getInstance();
//        init(robot);
//        runAction(robot);
//        kill(robot);
//    }

    public static void run() {
        Robot robot = Robot.getInstance();
        init(robot);
        runAction(robot);
        kill(robot);
    }

    public static void init(Robot robot) {
        robot.lift.getMotor().setTargetPosition(0);
    }

    public static void runAction(Robot robot) {
        robot.lift.getMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(0.2);
    }
//
//    private void isActionComplete() {
//
//    }

    public static void kill(Robot robot) {
        //actionIsComplete = true;
        robot.lift.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
