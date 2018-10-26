package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HWMaps.Robot;

@Autonomous(name="Park From Ground", group="Iterative Opmode")
public class LiftEncoderTest extends LinearOpMode {

    // Declare OpMode members.
    private Robot robot = Robot.getInstance();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

    }

}
