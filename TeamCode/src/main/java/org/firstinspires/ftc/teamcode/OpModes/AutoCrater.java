package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.HardwareMaps.Robot;
import org.firstinspires.ftc.teamcode.Library.RubiesLinearOpMode;
import org.firstinspires.ftc.teamcode.Library.TensorFlow;

@Autonomous(name="Crater Side Full", group="auto")
//@Disabled
public class AutoCrater extends RubiesLinearOpMode {
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
        robot.lift.lower();
        robot.drive.unlatch();
        robot.drive.stop();
        robot.sample(goldPosition);
        robot.drive.backupFromSampling(goldPosition);
        robot.drive.driveToWall(goldPosition);  //include turning to heading of 0
        robot.drive.alignWithWall(Robot.StartingPosition.CRATER);   //include turning to heading 45
        robot.drive.driveToDepot(Robot.StartingPosition.CRATER);
        robot.claim.depositTeamMarker();    //include deploy, wait, and then stow
        robot.park(robot.StartingPosition.CRATER);

//        if (goldPosition == TensorFlow.GoldPosition.RIGHT) {
//            robot.drive.setIndividualPowers(0.25, 0.25, 0.25, 0.25);
//            sleepFor(1900);
//            robot.drive.stop();
//            robot.turnToHeadingCenterPivot(0);
//            robot.drive.driveToWall(goldPosition);
//            robot.turnToHeadingCenterPivot(45);
//            robot.drive.alignWithWallCrater();
//            robot.drive.initializeTrajectory(0, -85, 45).run();
//            robot.claim.deploy();
//            sleepFor(2000);
//            robot.claim.stow();
//            robot.drive.initializeTrajectory(0, 145, 45).run();
//        } else if (goldPosition == TensorFlow.GoldPosition.LEFT){
//            robot.drive.setIndividualPowers(0.25, 0.25, 0.25, 0.25);
//            sleepFor(1600);
//            robot.drive.stop();
//            robot.turnToHeadingCenterPivot(0);
//            robot.drive.driveToWall(goldPosition);
//            robot.turnToHeadingCenterPivot(45);
//            robot.drive.alignWithWallCrater();
//            robot.drive.initializeTrajectory(0, -75, 45).run();
//            robot.claim.deploy();
//            sleepFor(2000);
//            robot.claim.stow();
//            robot.drive.initializeTrajectory(0, 145, 45).run();
//        } else {
//            robot.drive.setIndividualPowers(0.25, 0.25, 0.25, 0.25);
//            sleepFor(1700);
//            robot.drive.stop();
//            robot.turnToHeadingCenterPivot(0);
//            robot.drive.driveToWall(goldPosition);
//            robot.turnToHeadingCenterPivot(45);
//            robot.drive.alignWithWallCrater();
//            robot.drive.initializeTrajectory(0, -75, 45).run();
//            robot.claim.deploy();
//            sleepFor(2000);
//            robot.claim.stow();
//            robot.drive.initializeTrajectory(0, 160, 45).run();
//        }
    }
}
