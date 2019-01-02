package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HWMaps.Robot;

@TeleOp(name="Lower Test", group="Tests")
public class LowerTest extends LinearOpMode {
    private Robot robot = Robot.getInstance();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            robot.lift.lowerRobotToGround();
            telemetry.addData("Position", robot.lift.getMotor().getCurrentPosition());
            telemetry.update();
        }
    }

}
