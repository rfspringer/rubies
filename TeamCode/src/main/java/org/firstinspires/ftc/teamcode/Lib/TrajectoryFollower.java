package org.firstinspires.ftc.teamcode.Lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TrajectoryFollower {
    private ElapsedTime timer = new ElapsedTime();
    private double kV;
    private double kA;
    private boolean usesFeedback;
    private DcMotor[] motors;
    private TrajectoryGenerator trajectory;
    private boolean directionIsPositive;

    public TrajectoryFollower(DcMotor[] motors, TrajectoryGenerator trajectory, double kV, double kA, boolean usesFeedback, boolean directionIsNonNeg){
        this.usesFeedback = usesFeedback;
        this.motors = motors;
        this.trajectory = trajectory;
        this.kV = kV;
        this.kA = kA;
        this.directionIsPositive = directionIsPositive;
        timer.reset();
    }

    public void run(){
        if (usesFeedback) {
            //run PID with heading and feedforward
        } else {
            MotorEnhanced.setPower(motors, getFeedforwardPower(timer));
        }
    }

    private double getFeedforwardPower(ElapsedTime currentTime){
        trajectory.calculatePositionalDerivatives(currentTime);
        double power = kV * trajectory.getCurrentVelocity() + kA * trajectory.getCurrentAcceleration();
        if (directionIsPositive) {
            return power;
        } else {
            return -power;
        }
    }

    public boolean trajectoryIsComplete() {
        return timer.seconds() > 0.1 && trajectory.getCurrentVelocity() <= 0;
    }
}
