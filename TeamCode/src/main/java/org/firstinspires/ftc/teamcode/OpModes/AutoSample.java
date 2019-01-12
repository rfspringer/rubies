package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareMaps.Mineral;
import org.firstinspires.ftc.teamcode.HardwareMaps.Robot;
import org.firstinspires.ftc.teamcode.Library.Archived.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.Library.MecanumTrajectoryFollower;
import org.firstinspires.ftc.teamcode.Library.TensorFlow;

@Autonomous(name="Sample", group="auto")
//@Disabled
public class AutoSample extends LinearOpMode {
    // Declare OpMode members
    private Robot robot = Robot.getInstance();
    private TensorFlow tensorFlow = new TensorFlow();

    @Override
    public void runOpMode() {
        telemetry.addData("Instructions", "Initialize robot against phone-side wall");
        telemetry.update();
        robot.init(hardwareMap);
        tensorFlow.init(hardwareMap);
        robot.lift.holdHangingPosition();
        robot.drive.setInAutonomous(true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        tensorFlow.activate();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        TensorFlow.GoldPosition goldPos = tensorFlow.getGoldPos();
        tensorFlow.shutdown();
        robot.lift.lowerRobotToGround();
        robot.drive.unlatch();
        robot.turnToHeadingCenterPivot(0);
        robot.sample(goldPos);

    }
}
