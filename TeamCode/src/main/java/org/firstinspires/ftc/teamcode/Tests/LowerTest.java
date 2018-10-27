package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Actions.Hang;
import org.firstinspires.ftc.teamcode.Actions.Lower;
import org.firstinspires.ftc.teamcode.HWMaps.Robot;

@TeleOp(name="Lower Test", group="Tests")
public class LowerTest extends LinearOpMode {
    private Robot robot = Robot.getInstance();
    private Lower lower = new Lower(robot);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        lower.init();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            lower.run();
            if (robot.lift.getMotor().getCurrentPosition() < -3800) {
                lower.kill();
            }
            telemetry.addData("Position", robot.lift.getMotor().getCurrentPosition());
            telemetry.update();
        }
    }

}
