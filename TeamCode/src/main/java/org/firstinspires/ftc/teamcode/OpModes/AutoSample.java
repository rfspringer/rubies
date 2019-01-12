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
    private MecanumTrajectoryFollower mineralTraj;

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
        mineralTraj = adjustTrajectory();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        TensorFlow.GoldPosition goldPos = tensorFlow.getGoldPos();
        goldPos = TensorFlow.GoldPosition.CENTER;
        tensorFlow.shutdown();
        robot.lift.lowerRobotToGround();
        robot.drive.unlatch();
        robot.turnToHeadingCenterPivot(0);
//        robot.sample(goldPos);
        mineralTraj.run();
    }

    public MecanumTrajectoryFollower adjustTrajectory() {
        double x = 10;
        double y = -75;
        double heading = 0;
        ElapsedTime timer = new ElapsedTime();
            while (!isStarted()){
                if (gamepad1.dpad_up && timer.milliseconds() > 500) {
                    y += 1;
                    timer.reset();
                } else if (gamepad1.dpad_down && timer.milliseconds() > 500) {
                    y -= 1;
                    timer.reset();
                } else if (gamepad1.dpad_right && timer.milliseconds() > 500) {
                    x += 1;
                    timer.reset();
                } else if (gamepad1.dpad_left && timer.milliseconds() > 500) {
                    x -= 1;
                    timer.reset();
                }

                telemetry.addData("x", x);
                telemetry.addData("y", x);
                telemetry.update();
            }
        return robot.drive.initializeTrajectory(x, y, 0);
    }
}
