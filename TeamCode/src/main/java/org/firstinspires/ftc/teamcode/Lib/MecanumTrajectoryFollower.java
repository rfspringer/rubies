package org.firstinspires.ftc.teamcode.Lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HWMaps.MecanumRobot;

public class MecanumTrajectoryFollower {
    private MecanumRobot robot = MecanumRobot.getInstance();
    private MecanumEnhanced mecanumEnhanced = new MecanumEnhanced();
    private ElapsedTime timer = new ElapsedTime();
    private double targetHeading;
    private double power = 0;
    private double kA;
    private double x;
    private double y;
    private boolean usesFeedback = false;
    private boolean hasResetTimer = false;
    private DcMotor[] motors;
    private MecanumTrajectoryGenerator trajectory;

    public MecanumTrajectoryFollower(DcMotor[] motors, MecanumTrajectoryGenerator trajectory, double heading, double kV, double kA, boolean usesFeedback){
        this.usesFeedback = usesFeedback;
        this.motors = motors;
        this.trajectory = trajectory;
        this.x = trajectory.getX();
        this.y = trajectory.getY();
        this.kA = kA;
        this.targetHeading = heading;
        MotorEnhanced.setRunMode(motors, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void run(){
        while (!trajectoryIsComplete()) {
            if (!hasResetTimer) {
                timer.reset();
                hasResetTimer = true;
            }

            robot.drive.setPowers(getMagnitude(timer), x, y, targetHeading);
        }
        MotorEnhanced.setPower(motors, 0);
    }

    private double getMagnitude(ElapsedTime currentTime){
        trajectory.calculatePositionalDerivatives(currentTime);
        double kV = 0.8 / mecanumEnhanced.getMaxVel(1, x, y);
        return kV * trajectory.getCurrentVelocity() + kA * trajectory.getCurrentAcceleration();
    }

    public boolean trajectoryIsComplete() {
        return trajectory.getTotalTime() > timer.seconds();
    }
}
