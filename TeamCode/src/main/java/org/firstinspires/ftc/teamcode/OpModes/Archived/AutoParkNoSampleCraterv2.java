package org.firstinspires.ftc.teamcode.OpModes.Archived;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMaps.Archived.Robotv2;
import org.firstinspires.ftc.teamcode.Library.TrajectoryFollower;

@Autonomous(name="Auto Park No Sample Crater", group="Iterative Opmode")
@Disabled
public class AutoParkNoSampleCraterv2 extends LinearOpMode {

    // Declare OpMode members.
    private Robotv2 robot = Robotv2.getInstance();

    @Override
    public void runOpMode() {
        telemetry.addData("Instructions", "Initialize robot against phone-side wall");
        telemetry.update();
        robot.init(hardwareMap);
        robot.lift.holdHangingPosition();
        TrajectoryFollower driveAwayFromLatch = robot.drive.initializeTrajectory(-15, -30);
        TrajectoryFollower driveFromUnlatchedToDepot = robot.drive.initializeTrajectory(100, 180);
//        TrajectoryFollower driveAwayFromMarker = robot.drive.initializeTrajectory(-10, 30);

        telemetry.addData("Instructions", "Initialize robot against left wall");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Task", "Time to lower from the lander!");
        telemetry.update();
        robot.lift.lowerRobotToGround();
        telemetry.addData("Task", "Alrighty, now I'm gonna turn");
        telemetry.update();
        robot.turnToHeadingCenterPivot(-30);
        telemetry.addData("Task", "Now I'll drive out from the latch :)");
        telemetry.update();
        driveAwayFromLatch.run();
        robot.turnToHeadingCenterPivot(180);
        telemetry.addData("Task", "Time to head over to the depot");
        telemetry.update();
        driveFromUnlatchedToDepot.run();
        telemetry.addData("Status", "All done, go RUBOT!");
        telemetry.update();
    }
}
