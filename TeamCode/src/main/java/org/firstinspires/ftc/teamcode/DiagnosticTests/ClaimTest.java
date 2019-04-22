package org.firstinspires.ftc.teamcode.DiagnosticTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.Archived.Robotv3;
import org.firstinspires.ftc.teamcode.HardwareMaps.Robot;

@TeleOp(name="Claim Test", group="Tests")
public class ClaimTest extends LinearOpMode {
    private Robot robot = Robot.getInstance();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.claim.stow();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        robot.claim.deploy();
        sleep(2000);
    }
}
