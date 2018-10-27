package org.firstinspires.ftc.teamcode.Lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TrajectoryFollower {
    private ElapsedTime timer = new ElapsedTime();
    private double power = 0;
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
        MotorEnhanced.setRunMode(motors, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void run(){
        while (!trajectoryIsComplete()) {
            if (!hasResetTimer) {
                timer.reset();
                hasResetTimer = true;
            }
//            if (trajectoryIsComplete()) {
//                MotorEnhanced.setPower(motors, 0);
//            } else if (usesFeedback) {
//                //run PID with heading and feedforward
//            } else {
                power = getFeedforwardPower(timer);
                MotorEnhanced.setPower(motors, power);
//            }
        }
        MotorEnhanced.setPower(motors, 0);
    }

    private double getFeedforwardPower(ElapsedTime currentTime){
        trajectory.calculatePositionalDerivatives(currentTime);
        double power = kV * trajectory.getCurrentVelocity() + kA * trajectory.getCurrentAcceleration();
        return trajectory.getDirection() * power;
    }

    public boolean trajectoryIsComplete() {
        if (trajectory.getDirection() > 0) {
            return timer.seconds() > 0.1 && power <= 0;
        } else {
            return timer.seconds() > 0.1 && power >= 0;
        }
    }
}
