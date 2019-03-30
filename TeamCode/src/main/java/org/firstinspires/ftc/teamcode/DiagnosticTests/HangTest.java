package org.firstinspires.ftc.teamcode.DiagnosticTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.Archived.Robotv3;

@TeleOp(name="Hang Test", group="Tests")
public class HangTest extends LinearOpMode {
    private Robotv3 robot = Robotv3.getInstance();

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
