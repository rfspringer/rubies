package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Actions.Hang;
import org.firstinspires.ftc.teamcode.Actions.MineralToDepot;
import org.firstinspires.ftc.teamcode.Actions.Sample;
import org.firstinspires.ftc.teamcode.HWMaps.Robot;
import org.firstinspires.ftc.teamcode.Lib.TrajectoryFollower;

@Autonomous(name="Auto Park No Sample Depot", group="Iterative Opmode")
//@Disabled
public class AutoParkNoSampleDepot extends LinearOpMode {

    // Declare OpMode members.
    private Robot robot = Robot.getInstance();
    private Hang hang = new Hang(robot);
    private Sample sample = new Sample(robot);
    private MineralToDepot mineralToDepot = new MineralToDepot(robot);

    @Override
    public void runOpMode() {
        telemetry.addData("Instructions", "Initialize robot against phone-side wall");
        telemetry.update();
        robot.init(hardwareMap);
        hang.init();
        TrajectoryFollower driveAwayFromLatch = robot.drive.initializeTrajectory(-15, -30);
        TrajectoryFollower driveFromUnlatchedToDepot = robot.drive.initializeTrajectory(150, 180);
        TrajectoryFollower driveABitFurtherToDepot = robot.drive.initializeTrajectory(10, 180);
        TrajectoryFollower driveToCrater = robot.drive.initializeTrajectory(250, -45);

        telemetry.addData("Instructions", "Initialize robot against left wall");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!isStarted()) {
            hang.runAction();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        sample.init();
        hang.kill();
        telemetry.addData("Task", "Time to lower from the lander!");
        telemetry.update();
        robot.lift.lowerRobotToGround();
        telemetry.addData("Task", "Alrighty, now I'm gonna turn");
        telemetry.update();
        robot.turnToHeading(-30);
        telemetry.addData("Task", "Now I'll drive out from the latch :)");
        telemetry.update();
        driveAwayFromLatch.run();
        robot.turnToHeading(170);
        telemetry.addData("Task", "Time to head over to the depot");
        telemetry.update();
        driveFromUnlatchedToDepot.run();
        telemetry.addData("Task", "One last thing...");
        telemetry.update();
//        robot.turnToHeading(165);
        robot.claim.deploy();
        sleep(1500);
        robot.claim.stow();
//        robot.turnToHeading(180);
        driveABitFurtherToDepot.run();
        robot.turnToHeading(-43);
        driveToCrater.run();
        telemetry.addData("Status", "All done, go RUBOT!");
        telemetry.update();
    }
}
