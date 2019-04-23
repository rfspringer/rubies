package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.HardwareMaps.Mineral.MineralParent;

/**
 * This variation of our standard trajectory follower is used for arms
 * Takes into account the effect of gravity when setting powers
 */
public class PivotTrajectoryFollower extends TrajectoryFollower {
    private MineralParent mineral = MineralParent.getInstance();
    private ElapsedTime timer = new ElapsedTime();
    private double kAExternal;
    private RubiesLinearOpMode opMode;

    public PivotTrajectoryFollower(RubiesLinearOpMode opMode, DcMotor[] motors, TrajectoryGenerator trajectory, double kV, double kA, double kAExternal){
        super(opMode, motors, trajectory, kV, kA);
        this.kAExternal = kAExternal;
    }

    @Override
    public void run(){
        timer.reset();
        while (!trajectoryIsComplete()) {
            opMode.telemetry.addData("trajectory", "running");
            opMode.telemetry.update();
            double power = getFeedforwardPower(timer);
            mineral.setArmPower(power);
        }
        mineral.setArmPower(0);
    }

    protected double getFeedforwardPower(ElapsedTime currentTime){
        return super.getFeedforwardPower(currentTime) + kAExternal * mineral.getAngularAccelerationFromGravity();
    }
}
