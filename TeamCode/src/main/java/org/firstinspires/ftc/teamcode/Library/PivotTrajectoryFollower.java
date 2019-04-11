package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.HardwareMaps.Mineral.MineralParent;

public class PivotTrajectoryFollower extends TrajectoryFollower {
    private MineralParent mineral = MineralParent.getInstance();
    private ElapsedTime timer = new ElapsedTime();
    private double kAExternal;

    public PivotTrajectoryFollower(DcMotor[] motors, TrajectoryGenerator trajectory, double kV, double kA, double kAExternal){
        super(motors, trajectory, kV, kA);
        this.kAExternal = kAExternal;
    }

    @Override
    public void run(){
        timer.reset();
        while (!trajectoryIsComplete()) {
            double power = getFeedforwardPower(timer);
            mineral.setArmPower(power);
        }
        mineral.setArmPower(0);
    }

    protected double getFeedforwardPower(ElapsedTime currentTime){
        return super.getFeedforwardPower(currentTime) + kAExternal * mineral.getAngularAccelerationFromGravity();
    }
}
