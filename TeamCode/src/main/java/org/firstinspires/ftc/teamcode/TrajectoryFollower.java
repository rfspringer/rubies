package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TrajectoryFollower {
    private double kV;
    private double kA;
    private boolean usesFeedback;
    private DcMotor[] motors;

    public TrajectoryFollower(boolean usesFeedback, DcMotor[] motors){
        this.usesFeedback = usesFeedback;
        this.motors = motors;
    }

    public void run(){
        if (usesFeedback) {
            //run PID with current position and feedforward
        } else {
            MotorEnhanced.setPowers(motors, getFeedforwardPower());
        }
    }
}
