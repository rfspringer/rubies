package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMaps.Robot;
import org.firstinspires.ftc.teamcode.Library.MecanumTrajectoryFollower;
import org.firstinspires.ftc.teamcode.Library.TensorFlow;

@Autonomous(name="Claim", group="auto")
//@Disabled
public class AutoClaim extends LinearOpMode {
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
        tensorFlow.activate();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }
        TensorFlow.GoldPosition goldPos = tensorFlow.getGoldPos();
        tensorFlow.shutdown();
        robot.lift.lowerRobotToGround();
        robot.drive.unlatch();
        robot.turnToHeadingCenterPivot(0);
        robot.sample(goldPos);
        robot.goToDepot(goldPos);
        robot.claim.deploy();
        sleep(2000);
        robot.claim.stow();
        robot.drive.setIndividualPowers(-0.5, -0.5, -0.5, -0.5);
        sleep(500);
        robot.drive.stop();
    }
}
