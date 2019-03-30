package org.firstinspires.ftc.teamcode.DiagnosticTests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMaps.Robot;
import org.firstinspires.ftc.teamcode.Library.RubiesLinearOpMode;

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
