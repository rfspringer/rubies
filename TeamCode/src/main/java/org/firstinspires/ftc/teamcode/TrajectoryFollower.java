package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TrajectoryFollower {
    private ElapsedTime timer = new ElapsedTime();
    private double kV;
    private double kA;
    private boolean usesFeedback;
    private DcMotor[] motors;
    private TrajectoryGenerator trajectory;

    public TrajectoryFollower(DcMotor[] motors, TrajectoryGenerator trajectory, double kV, double kA, boolean usesFeedback){
        this.usesFeedback = usesFeedback;
        this.motors = motors;
        this.trajectory = trajectory;
        this.kV = kV;
        this.kA = kA;
        timer.reset();
    }

    public void run(){
        if (usesFeedback) {
            //run PID with heading and feedforward
        } else {
            MotorEnhanced.setPowers(motors, getFeedforwardPower(timer));
        }
    }

    private double getFeedforwardPower(ElapsedTime currentTime){
        trajectory.calculatePositionalDerivatives(currentTime);
        return kV * trajectory.getCurrentVelocity() + kA * trajectory.getCurrentAcceleration();
    }

    public boolean trajectoryIsComplete() {
        return timer.seconds() > 0.1 && trajectory.getCurrentVelocity() <= 0;
    }
}
