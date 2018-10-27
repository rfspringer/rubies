package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Actions.Hang;
import org.firstinspires.ftc.teamcode.Actions.Lower;
import org.firstinspires.ftc.teamcode.HWMaps.Robot;
import org.firstinspires.ftc.teamcode.HWMaps.Sensors;
import org.firstinspires.ftc.teamcode.Lib.TrajectoryFollower;

@Autonomous(name="Park From Ground", group="Iterative Opmode")
//@Disabled
public class AutoNoPark extends LinearOpMode {

    // Declare OpMode members.
    private Robot robot = Robot.getInstance();
    private Hang hang = new Hang(robot);
    private Lower lower = new Lower(robot);

    private double LEFT_MINERAL_HEADING;
    private double RIGHT_MINERAL HEADING

    @Override
    public void runOpMode() {
        telemetry.addData("Instructions", "Initialize robot against left wall");
        telemetry.update();
        robot.init(hardwareMap);
        hang.init();
        TrajectoryFollower driveAwayFromLatch = robot.drive.initializeTrajectory(36, 30);
        TrajectoryFollower  driveToCenterMineral = robot.drive.initializeTrajectory(36, 0);
        TrajectoryFollower  driveToLeftMineral = robot.drive.initializeTrajectory(48, -50);
        TrajectoryFollower  driveToRightMineral = robot.drive.initializeTrajectory(48, 50);

        telemetry.addData("Instructions", "Initialize robot against left wall");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!isStarted()) {
            hang.run();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        hang.kill();
        lower.init();
        lower.run();
        lower.kill();
        robot.turnToHeading(30);
//        driveAwayFromLatch.run();
//        robot.turnToHeading(0);
//        if (robot.sensors.getGoldPosition() == Sensors.GoldLocation.LEFT) {
//
//        } else if (robot.sensors.getGoldPosition() == Sensors.GoldLocation.RIGHT) {
//
//        } else {
//
//        }

    }
}
