package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Actions.Hang;
import org.firstinspires.ftc.teamcode.HWMaps.Robot;

@TeleOp(name="HangTest", group="Iterative Opmode")
public class HangTest extends LinearOpMode {
    private Robot robot = Robot.getInstance();
    private Hang hang = new Hang();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        hang.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            hang.run();
        }
    }

}
