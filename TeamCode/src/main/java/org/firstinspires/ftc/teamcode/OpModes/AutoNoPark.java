package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Actions.Hang;
import org.firstinspires.ftc.teamcode.Actions.Lower;
import org.firstinspires.ftc.teamcode.Actions.MineralToDepot;
import org.firstinspires.ftc.teamcode.Actions.Sample;
import org.firstinspires.ftc.teamcode.HWMaps.Robot;
import org.firstinspires.ftc.teamcode.HWMaps.Sensors;
import org.firstinspires.ftc.teamcode.Lib.TrajectoryFollower;

@Autonomous(name="Auto No Park", group="Iterative Opmode")
//@Disabled
public class AutoNoPark extends LinearOpMode {

    // Declare OpMode members.
    private Robot robot = Robot.getInstance();
    private Hang hang = new Hang(robot);
    private Lower lower = new Lower(robot);
    private Sample sample = new Sample(robot);
    private MineralToDepot mineralToDepot = new MineralToDepot(robot);

    @Override
    public void runOpMode() {
        telemetry.addData("Instructions", "Initialize robot against phone-side wall");
        telemetry.update();
        robot.init(hardwareMap);
        hang.init();
        TrajectoryFollower driveAwayFromLatch = robot.drive.initializeTrajectory(-36, 30);
        TrajectoryFollower driveAwayFromMarker = robot.drive.initializeTrajectory(10, 30);

        telemetry.addData("Instructions", "Initialize robot against left wall");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while (!isStarted()) {
            hang.run();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        sample.init();
        hang.kill();
        lower.init();
        telemetry.addData("Task", "Time to lower from the lander!");
        telemetry.update();
        lower.run();
        lower.kill();
        telemetry.addData("Task", "Alrighty, now I'm gonna turn");
        telemetry.update();
//        robot.turnToHeading(-30);
        telemetry.addData("Task", "Now I'll drive out from the latch :)");
        telemetry.update();
//        driveAwayFromLatch.run();
        telemetry.addData("Task", "I'm gonna sample!");
        telemetry.update();
//        sample.run();
        telemetry.addData("Task", "Time to head over to the depot");
        telemetry.update();
//        mineralToDepot.init();
//        mineralToDepot.run();
        telemetry.addData("Task", "One last thing...");
        telemetry.update();
//        robot.claim.deploy();
//        sleep(1500);
//        robot.claim.stow();
//        driveAwayFromMarker.run();
//        telemetry.addData("Status", "All done, go RUBOT!");
//        telemetry.update();
    }
}
