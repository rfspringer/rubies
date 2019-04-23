package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Follows a trajectory with inputted motors based on vel and accel constants
 */
public class TrajectoryFollower {
    private ElapsedTime timer = new ElapsedTime();
    private boolean hasResetTimer = false;
    private double externalAcceleration = 0;
    private double kV;
    private double kA;
    private double kAExternal;
    private DcMotor[] motors;
    private TrajectoryGenerator trajectory;
    private RubiesLinearOpMode rubiesLinearOpMode;

    public TrajectoryFollower(RubiesLinearOpMode opMode, DcMotor[] motors, TrajectoryGenerator trajectory, double kV, double kA){
        this.rubiesLinearOpMode = opMode;
        this.motors = motors;
        this.trajectory = trajectory;
        this.kV = kV;
        this.kA = kA;
        MotorEnhanced.setRunMode(motors, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public TrajectoryFollower(RubiesLinearOpMode opMode, DcMotor[] motors, TrajectoryGenerator trajectory, double kV, double kA, double externalAcceleration, double kAExternal){
        this.externalAcceleration = externalAcceleration;
        this.kAExternal = kAExternal;
        new TrajectoryFollower(opMode, motors, trajectory, kA, kV);
    }

    public void run(){
        timer.reset();
        while (!trajectoryIsComplete()) {
            rubiesLinearOpMode.telemetry.addData("velocity", trajectory.getCurrentVelocity());
            rubiesLinearOpMode.telemetry.update();
            double power = getFeedforwardPower(timer);
            MotorEnhanced.setPower(motors, power);
        }
        MotorEnhanced.setPower(motors, 0);
    }

    protected double getFeedforwardPower(ElapsedTime currentTime){
        trajectory.calculatePositionalDerivatives(currentTime);
        return kV * trajectory.getCurrentVelocity() + kA * trajectory.getCurrentAcceleration() + kAExternal * externalAcceleration;
    }

    public boolean trajectoryIsComplete() {
        return trajectory.getCurrentVelocity() < 0;
    }
}
