package org.firstinspires.ftc.teamcode.OpModes.Archived;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.HardwareMaps.Archived.Robotv2;

@Autonomous(name="Park From Ground", group="Iterative Opmode")
@Disabled
public class AutoParkv2 extends LinearOpMode {

    // Declare OpMode members.
    private Robotv2 robot = Robotv2.getInstance();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.drive.setPowers(0.5, 0.5);
        sleep(2100);
        robot.drive.setPowers(0, 0);
    }
}
