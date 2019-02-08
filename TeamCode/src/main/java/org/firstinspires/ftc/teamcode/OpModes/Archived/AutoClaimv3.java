package org.firstinspires.ftc.teamcode.OpModes.Archived;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareMaps.Archived.Robotv3;
import org.firstinspires.ftc.teamcode.HardwareMaps.Robot;
import org.firstinspires.ftc.teamcode.Library.MecanumTrajectoryFollower;
import org.firstinspires.ftc.teamcode.Library.TensorFlow;

@Autonomous(name="Claimv3", group="auto")
@Disabled
public class AutoClaimv3 extends LinearOpMode {
    // Declare OpMode members
    private Robotv3 robot = Robotv3.getInstance();
    private TensorFlow tensorFlow = new TensorFlow();
    private ElapsedTime sleepTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Instructions", "Initialize robot against phone-side wall");
        telemetry.update();
        robot.init(hardwareMap);
        tensorFlow.init(hardwareMap);
        robot.lift.holdHangingPosition();
        robot.drive.setInAutonomous(true);
        tensorFlow.activate();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }
        TensorFlow.GoldPosition goldPos = tensorFlow.getGoldPos();
        telemetry.addData("Gold pos", goldPos);
        telemetry.update();
        tensorFlow.shutdown();
        robot.lift.lowerRobotToGround();
        robot.drive.unlatch();
        if (goldPos == TensorFlow.GoldPosition.RIGHT) {
            sleepFor(15000);
        }
        robot.turnToHeadingCenterPivot(0);
        robot.sample(goldPos);
        robot.goToDepot(goldPos);
        robot.claim.deploy();
        sleepFor(2000);
        robot.claim.stow();
        robot.drive.setIndividualPowers(-0.5, -0.5, -0.5, -0.5);
        sleepFor(500);
        robot.drive.stop();
    }

    private void sleepFor(double milliseconds) {
        sleepTimer.reset();
        while (sleepTimer.milliseconds() < milliseconds) {
            telemetry.addData("Status", "sleeping");
            telemetry.update();
        }
    }
}
