package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.HardwareMaps.Robot;
import org.firstinspires.ftc.teamcode.Library.RubiesLinearOpMode;

@TeleOp(name="DOE Program", group="Tests")
public class DOE extends RubiesLinearOpMode {
    private Robot robot = Robot.getInstance();

    private double intakePower = 1;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!isStarted()) {
            if (gamepad1.a) {
                intakePower = 0.5;
            }
            telemetry.addData("Instructions", "Press A to set intake power to 0.5");
            telemetry.addData("Intake power", intakePower);
            telemetry.update();
        }

        robot.drive.setPowers(0.75, 0, 1, 0);
        robot.mineral.setIntakeScaledPower(intakePower);
        sleepFor(750);
        robot.drive.stop();
        robot.mineral.setIntakeScaledPower(0);
    }
}
