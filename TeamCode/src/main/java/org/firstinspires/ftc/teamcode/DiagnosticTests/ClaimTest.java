package org.firstinspires.ftc.teamcode.DiagnosticTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.Archived.Robotv3;

@TeleOp(name="Claimv3 Test", group="Tests")
public class ClaimTest extends LinearOpMode {
    private Robotv3 robot = Robotv3.getInstance();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        robot.claim.deploy();
        sleep(2000);
    }
}
