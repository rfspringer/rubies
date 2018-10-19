package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TrajectoryGenerator {
    // Units are inches and seconds
    private double maxVelocity;
    private double maxAcceleration;

    private double trajectoryLength;

    public TrajectoryGenerator(double trajectoryLength, double maxVelocity, double maxAcceleration) {
        this.trajectoryLength = trajectoryLength;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    public double calculateVelocityAtTime(ElapsedTime currentTime) {
        double velocityWithoutCruising = Math.min(velocityIfConstantAcceleration(currentTime),
                velocityIfConstantDeceleration(currentTime));
        return Math.min(velocityWithoutCruising, maxVelocity);
    }

    private double velocityIfConstantAcceleration(ElapsedTime currentTime) {
        return maxAcceleration * currentTime.seconds();
    }

    private double velocityIfConstantDeceleration(ElapsedTime currentTime) {
        return trajectoryLength - maxAcceleration * currentTime.seconds();
    }
}
