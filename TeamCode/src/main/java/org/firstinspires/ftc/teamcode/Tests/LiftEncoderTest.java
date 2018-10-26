package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HWMaps.Robot;

@Autonomous(name="Lift Encoder Test", group="Tests")
public class LiftEncoderTest extends LinearOpMode {

    // Declare OpMode members.
    private Robot robot = Robot.getInstance();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "This code will display the encoder reading of the lift motor, which can be moved by hand to certain positions");
        telemetry.update();

        while (opModeIsActive()) {
            telemetry.addData("Lift Encoder Pos", robot.lift.getEncoderCounts());
            telemetry.update();
        }
    }

}
