package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareMaps.Archived.Robotv3;
import org.firstinspires.ftc.teamcode.HardwareMaps.Robot;
import org.firstinspires.ftc.teamcode.Library.RubiesLinearOpMode;
import org.firstinspires.ftc.teamcode.Library.TensorFlow;

@Autonomous(name="Sample", group="auto")
//@Disabled
public class AutoSample extends RubiesLinearOpMode {
    private Robot robot = Robot.getInstance();
    private TensorFlow tensorFlow = new TensorFlow();

    @Override
    public void runOpMode() {
        telemetry.addData("Instructions", "Initialize robot against tape");
        telemetry.update();

        robot.init(hardwareMap);
        tensorFlow.init(hardwareMap);
        robot.drive.setInAutonomous(true);
        tensorFlow.activate();

        waitForStart();
        TensorFlow.GoldPosition goldPosition = tensorFlow.getGoldPos();
        tensorFlow.shutdown();
        robot.lift.lower();
        robot.drive.unlatch();
        robot.drive.stop();
        robot.sample(goldPosition);
    }
}
