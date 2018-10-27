package org.firstinspires.ftc.teamcode.Actions;

import org.firstinspires.ftc.teamcode.HWMaps.Robot;
import org.firstinspires.ftc.teamcode.HWMaps.Sensors;
import org.firstinspires.ftc.teamcode.Lib.TrajectoryFollower;

public class MineralToDepot extends Action {
    Robot robot;
    private Sensors.GoldLocation goldLocation;

    TrajectoryFollower driveFromCenterMineral;
    TrajectoryFollower driveFromLeftMineral;
    TrajectoryFollower driveFromRightMineral;

    private double LEFT_MINERAL_HEADING = -50;
    private double RIGHT_MINERAL_HEADING = 50;

    public MineralToDepot(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void init() {
        driveFromCenterMineral = robot.drive.initializeTrajectory(-36, 0);
        driveFromLeftMineral = robot.drive.initializeTrajectory(-48, LEFT_MINERAL_HEADING);
        driveFromRightMineral = robot.drive.initializeTrajectory(-48, RIGHT_MINERAL_HEADING);
    }

    @Override
    public void run() {
        if (robot.sensors.getHeading() < -10) {
            robot.turnToHeading(LEFT_MINERAL_HEADING);
            driveFromLeftMineral.run();
        } else if (robot.sensors.getHeading() > 10) {
            robot.turnToHeading(RIGHT_MINERAL_HEADING);
            driveFromRightMineral.run();
        } else {
            robot.turnToHeading(0);
            driveFromCenterMineral.run();
        }
    }

    @Override
    public void kill() {

    }
}
