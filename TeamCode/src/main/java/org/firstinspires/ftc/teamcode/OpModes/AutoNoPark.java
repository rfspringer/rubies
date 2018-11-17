package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HWMaps.Robot;
import org.firstinspires.ftc.teamcode.Lib.TensorFlow;
import org.firstinspires.ftc.teamcode.HWMaps.Sensors;
import org.firstinspires.ftc.teamcode.Lib.TrajectoryFollower;

@Autonomous(name="Auto No Park", group="Iterative Opmode")
//@Disabled
public class AutoNoPark extends LinearOpMode {
    // Declare OpMode members.
    private Robot robot = Robot.getInstance();

    @Override
    public void runOpMode() {
        telemetry.addData("Instructions", "Initialize robot against phone-side wall");
        telemetry.update();
        robot.init(hardwareMap);
        robot.lift.holdHangingPosition();
        TrajectoryFollower driveAwayFromLatch = robot.drive.initializeTrajectory(-15, -30);
        TrajectoryFollower driveFromUnlatchedToDepot = robot.drive.initializeTrajectory(150, 180);
        TrajectoryFollower driveAwayFromMarker = robot.drive.initializeTrajectory(-10, 30);

        telemetry.addData("Instructions", "Initialize robot against left wall");
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        Sensors.GoldLocation goldLocation = robot.sensors.getGoldPosition();
        telemetry.addData("Task", "Time to lower from the lander!");
        telemetry.update();
        robot.lift.lowerRobotToGround();
        telemetry.addData("Task", "Alrighty, now I'm gonna turn");
        telemetry.update();
        robot.turnToHeading(-30);
        telemetry.addData("Task", "Now I'll drive out from the latch :)");
        telemetry.update();
        driveAwayFromLatch.run();
        robot.turnToHeading(180);
        telemetry.addData("Task", "I'm gonna sample!");
        telemetry.update();
        robot.sample(goldLocation);
//        telemetry.addData("Task", "Time to head over to the depot");
//        telemetry.update();
//        driveFromUnlatchedToDepot.runAction();
////        mineralToDepot.init();
////        mineralToDepot.runAction();
//        telemetry.addData("Task", "One last thing...");
//        telemetry.update();
//        robot.claim.deploy();
//        sleep(1500);
//        robot.claim.stow();
//        driveAwayFromMarker.runAction();
//        telemetry.addData("Status", "All done, go RUBOT!");
//        telemetry.update();
    }
}
