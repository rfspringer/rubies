package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.HardwareMaps.Robot;
import org.firstinspires.ftc.teamcode.Library.RubiesLinearOpMode;
import org.firstinspires.ftc.teamcode.Library.TensorFlow;

@Autonomous(name="Depot Side Full", group="auto")
//@Disabled
public class AutoDepot extends RubiesLinearOpMode {
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
        telemetry.addData("Gold Position", goldPosition);
        telemetry.update();

//        robot.lift.lower();
//        robot.drive.unlatch();
//        robot.drive.stop();
        robot.sample(goldPosition);
        if (goldPosition == TensorFlow.GoldPosition.RIGHT) {
            robot.drive.setIndividualPowers(0.25, 0.25, 0.25, 0.25);
            sleepFor(1750);
            robot.drive.stop();
            robot.turnToHeadingCenterPivot(0);
            robot.drive.driveToWall(goldPosition);
            robot.turnToHeadingCenterPivot(45);
            robot.drive.alignWithWallDepot();
            robot.drive.initializeTrajectory(0, 85, 45).run();
            robot.claim.deploy();
            sleepFor(2000);
            robot.claim.stow();
            robot.drive.initializeTrajectory(0, -120, 45).run();
        } else if (goldPosition == TensorFlow.GoldPosition.LEFT){
            robot.drive.setIndividualPowers(0.25, 0.25, 0.25, 0.25);
            sleepFor(1500);
            robot.drive.stop();
            robot.turnToHeadingCenterPivot(0);
            robot.drive.driveToWall(goldPosition);
            robot.turnToHeadingCenterPivot(45);
            robot.drive.alignWithWallDepot();
            robot.drive.initializeTrajectory(0, 75, 45).run();
            robot.claim.deploy();
            sleepFor(2000);
            robot.claim.stow();
            robot.drive.initializeTrajectory(0, -120, 45).run();
        } else {
            robot.drive.setIndividualPowers(0.25, 0.25, 0.25, 0.25);
            sleepFor(1500);
            robot.drive.stop();
            robot.turnToHeadingCenterPivot(0);
            robot.drive.driveToWall(goldPosition);
            robot.turnToHeadingCenterPivot(-135);
            robot.drive.alignWithWallDepot();
            robot.drive.initializeTrajectory(0, -75, -135).run();
            robot.claim.deploy();
            sleepFor(2000);
            robot.claim.stow();
            robot.drive.initializeTrajectory(0, 50, -135).run();
        }
    }
}
