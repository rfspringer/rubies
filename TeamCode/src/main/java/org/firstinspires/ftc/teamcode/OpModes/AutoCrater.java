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

        robot.init(hardwareMap, this);
        tensorFlow.init(hardwareMap);
        robot.drive.setInAutonomous(true);
        tensorFlow.activate();

        waitForStart();
        TensorFlow.GoldPosition goldPosition = tensorFlow.getGoldPos();
        tensorFlow.shutdown();
        telemetry.addData("Gold Position", goldPosition);
        telemetry.update();
        robot.lift.lower();
        robot.turnToHeadingCenterPivot(0);
        robot.drive.unlatch();
        robot.sample(goldPosition);
        robot.backupFromSampling(goldPosition);
        robot.drive.driveToWall(goldPosition);
        robot.alignWithWall(Robot.StartingPosition.CRATER);
        robot.drive.driveToDepot(Robot.StartingPosition.CRATER);
        robot.claim.deploy();
        robot.mineral.setExtensionPower(-1);
        robot.drive.park(Robot.StartingPosition.CRATER);
        robot.claim.stow();
        robot.mineral.setExtensionPower(0);
    }
}
