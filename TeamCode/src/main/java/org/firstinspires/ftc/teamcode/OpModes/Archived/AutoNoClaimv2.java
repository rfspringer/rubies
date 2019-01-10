package org.firstinspires.ftc.teamcode.OpModes.Archived;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMaps.Archived.Robotv2;
import org.firstinspires.ftc.teamcode.Library.TensorFlow;
import org.firstinspires.ftc.teamcode.Library.Archived.TrajectoryFollower;

@Autonomous(name="Auto No Claim", group="Iterative Opmode")
@Disabled
public class AutoNoClaimv2 extends LinearOpMode {
    // Declare OpMode members
    private Robotv2 robot = Robotv2.getInstance();
    private TensorFlow tensorFlow = new TensorFlow();

    @Override
    public void runOpMode() {
        telemetry.addData("Instructions", "Initialize robot against phone-side wall");
        telemetry.update();
        robot.init(hardwareMap);
        tensorFlow.init(hardwareMap);
        robot.lift.holdHangingPosition();
        TrajectoryFollower driveAwayFromLatch = robot.drive.initializeTrajectory(-15, -30);
        TrajectoryFollower driveFromUnlatchedToDepot = robot.drive.initializeTrajectory(150, 180);
        TrajectoryFollower driveALittleExtra = robot.drive.initializeTrajectory(-10, 30);

        telemetry.addData("Instructions", "Initialize robot against left wall");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        tensorFlow.activate();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        TensorFlow.GoldPosition goldPos = tensorFlow.getGoldPos();
        tensorFlow.shutdown();
        telemetry.addData("Task", "Time to lower from the lander!");
        telemetry.update();
        robot.lift.lowerRobotToGround();
        telemetry.addData("Task", "Alrighty, now I'm gonna turn");
        telemetry.update();
        robot.turnToHeadingCenterPivot(-30);
        telemetry.addData("Task", "Now I'll drive out from the latch :)");
        telemetry.update();
        driveAwayFromLatch.run();
        telemetry.addData("Task", "I'm gonna sample!");
        telemetry.addData("Mineral", goldPos);
        telemetry.update();
        robot.sample(goldPos);
        robot.drive.setPowers(-0.3, -0.3);
        sleep(750);
        robot.drive.setPowers(0, 0);
        telemetry.addData(">", "All done!!!");
    }
}
