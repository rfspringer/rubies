package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HWMaps.Archived.Robotv2;

@TeleOp(name="Lift Encoder Test", group="Tests")
public class LiftEncoderTest extends LinearOpMode {
    private Robotv2 robot = Robotv2.getInstance();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "This code will display the encoder reading of the lift motor, which can be moved by hand to certain positions");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Lift Encoder Pos", robot.lift.getEncoderCounts());
            telemetry.update();
        }
    }

}
