package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TrajectoryFollower {
    private ElapsedTime timer = new ElapsedTime();
    private double kV;
    private double kA;
    private boolean usesFeedback;
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
//
//    public TrajectoryFollower(TrajectoryGenerator trajectory, boolean usesFeedback){
//        // default trajectory follower for drivetrain
//        Drive drive = Drive.getInstance();
//        this.usesFeedback = usesFeedback;
//        this.motors = drive.getAllMotors();
//        this.trajectory = trajectory;
//        this.kV = drive.getkV();
//        this.kA = drive.getkA();
//    }

    public void run(){
        updateTimerIfFirstTime();
        if (usesFeedback) {
            //run PID with current position and feedforward
        } else {
            MotorEnhanced.setPowers(motors, getFeedforwardPower(timer));
        }
    }

    private void updateTimerIfFirstTime(){
        if (!hasResetTimer) {
            timer.reset();
            hasResetTimer = true;
        }
    }

    private double getFeedforwardPower(ElapsedTime currentTime){
        trajectory.calculatePositionalDerivatives(currentTime);
        return kV * trajectory.getCurrentVelocity() + kA * trajectory.getCurrentAcceleration();
    }
}
