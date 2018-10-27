package org.firstinspires.ftc.teamcode.Lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TrajectoryFollower {
    private ElapsedTime timer = new ElapsedTime();
    private double kV;
    private double kA;
    private boolean usesFeedback = false;
    private boolean hasResetTimer = false;
    private DcMotor[] motors;
    private TrajectoryGenerator trajectory;

    public TrajectoryFollower(DcMotor[] motors, TrajectoryGenerator trajectory, double kV, double kA, boolean usesFeedback){
        this.usesFeedback = usesFeedback;
        this.motors = motors;
        this.trajectory = trajectory;
        this.kV = kV;
        this.kA = kA;
    }

    public void run(){
        if (!hasResetTimer) {
            timer.reset();
            hasResetTimer = true;
        }
        if (usesFeedback) {
            //run PID with heading and feedforward
        } else {
            MotorEnhanced.setPower(motors, getFeedforwardPower(timer));
        }
    }

    private double getFeedforwardPower(ElapsedTime currentTime){
        trajectory.calculatePositionalDerivatives(currentTime);
        double power = kV * trajectory.getCurrentVelocity() + kA * trajectory.getCurrentAcceleration();
        if (trajectory.getDirection() > 0) {
            return power;
        } else {
            return -power;
        }
    }

    public boolean trajectoryIsComplete() {
        return timer.seconds() > 0.1 && trajectory.getCurrentVelocity() <= 0;
    }
}
