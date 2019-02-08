package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareMaps.Robot;
import org.firstinspires.ftc.teamcode.Library.RubiesLinearOpMode;
import org.firstinspires.ftc.teamcode.Library.TensorFlow;

@TeleOp(name="Unlatch Test", group="tests")
//@Disabled
public class UnlatchTest extends RubiesLinearOpMode {
    // Declare OpMode members
    private Robot robot = Robot.getInstance();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.drive.setInAutonomous(true);
        waitForStart();
        robot.drive.unlatch();
        robot.drive.stop();
        robot.turnToHeadingCenterPivot(45);
    }
}
