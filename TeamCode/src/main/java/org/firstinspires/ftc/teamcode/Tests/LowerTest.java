package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HWMaps.Archived.Robotv2;

@TeleOp(name="Lower Test", group="Tests")
public class LowerTest extends LinearOpMode {
    private Robotv2 robot = Robotv2.getInstance();

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
