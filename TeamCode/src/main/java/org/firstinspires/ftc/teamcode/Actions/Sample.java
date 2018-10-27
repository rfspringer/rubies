package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HWMaps.Robot;
import org.firstinspires.ftc.teamcode.HWMaps.Sensors;
import org.firstinspires.ftc.teamcode.Lib.TrajectoryFollower;

public class Sample extends Action {
    Robot robot;
    private Sensors.GoldLocation goldLocation;

    TrajectoryFollower driveToCenterMineral;
    TrajectoryFollower  driveToLeftMineral;
    TrajectoryFollower  driveToRightMineral;

    private double LEFT_MINERAL_HEADING = -45;
    private double RIGHT_MINERAL_HEADING = 45;

    public Sample(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        driveToCenterMineral = robot.drive.initializeTrajectory(-36, 0);
        driveToLeftMineral = robot.drive.initializeTrajectory(-48, LEFT_MINERAL_HEADING);
        driveToRightMineral = robot.drive.initializeTrajectory(-48, RIGHT_MINERAL_HEADING);
        goldLocation = robot.sensors.getGoldPosition();
    }

    @Override
    public void run() {
        if (goldLocation == Sensors.GoldLocation.LEFT) {
            robot.turnToHeading(LEFT_MINERAL_HEADING);
            driveToLeftMineral.run();
        } else if (goldLocation == Sensors.GoldLocation.RIGHT) {
            robot.turnToHeading(RIGHT_MINERAL_HEADING);
            driveToRightMineral.run();
        } else {
            robot.turnToHeading(0);
            driveToCenterMineral.run();
        }
    }

    @Override
    public void kill() {

    }
}
