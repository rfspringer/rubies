package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareMaps.Robot;
import org.firstinspires.ftc.teamcode.Library.TensorFlow;

@Autonomous(name="Unlatch Test", group="tests")
//@Disabled
public class UnlatchTest extends LinearOpMode {
    // Declare OpMode members
    private Robot robot = Robot.getInstance();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.drive.setInAutonomous(true);
        waitUntilStarted();
        robot.unlatch();
        robot.drive.stop();
    }

    private void waitUntilStarted() {
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }
    }
}
