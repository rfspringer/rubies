package org.firstinspires.ftc.teamcode.DiagnosticTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.Robot;

@TeleOp(name="Liftv3 Encoder Test", group="Tests")
public class LiftEncoderTest extends LinearOpMode {
    private Robot robot = Robot.getInstance();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "This code will display the encoder reading of the lift motor, which can be moved by hand to certain positions");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Liftv3 Encoder Pos", robot.lift.getMotor().getCurrentPosition());
            telemetry.update();
        }
    }
}
