package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto", group="Iterative Opmode")
public class park extends LinearOpMode {

    // Declare OpMode members.
    private RobotHardwareMap robot = RobotHardwareMap.getInstance();

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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

        }
    }

}
