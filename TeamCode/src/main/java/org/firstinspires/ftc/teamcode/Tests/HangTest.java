package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.Robot;

@TeleOp(name="Hang Test", group="Tests")
public class HangTest extends LinearOpMode {
    private Robot robot = Robot.getInstance();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.lift.holdHangingPosition();
        waitForStart();
        robot.lift.holdHangingPosition();
        while (opModeIsActive()) {

        }
    }
}
