package org.firstinspires.ftc.teamcode.OpModes.Archived;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMaps.Archived.Robotv3;
import org.firstinspires.ftc.teamcode.HardwareMaps.Robot;
import org.firstinspires.ftc.teamcode.Library.TensorFlow;

@Autonomous(name="Park", group="auto")
//@Disabled
public class AutoParkv3 extends LinearOpMode {
    // Declare OpMode members
    private Robotv3 robot = Robotv3.getInstance();
    private TensorFlow tensorFlow = new TensorFlow();

    @Override
    public void runOpMode() {
        telemetry.addData("Instructions", "Initialize robot against phone-side wall");
        telemetry.update();
        robot.init(hardwareMap);
        tensorFlow.init(hardwareMap);
//        robot.lift.kindaHoldHangingPosition();
//        while (!gamepad1.a) {
//            telemetry.addData("Status", "waiting for a");
//            telemetry.update();
//        }
        robot.lift.holdHangingPosition();
        robot.drive.setInAutonomous(true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        tensorFlow.activate();


        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Initialized");
            telemetry.update();
        }
        TensorFlow.GoldPosition goldPos = tensorFlow.getGoldPos();
        telemetry.addData("Gold pos", goldPos);
        tensorFlow.shutdown();
        robot.lift.lowerRobotToGround();
        robot.drive.unlatch();
        robot.sample(goldPos);
        robot.claim.deploy();
    }
}
