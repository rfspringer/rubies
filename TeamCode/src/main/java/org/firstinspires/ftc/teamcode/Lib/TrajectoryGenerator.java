package org.firstinspires.ftc.teamcode.Lib;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TrajectoryGenerator {
    // Units are inches and seconds
    private double maxVelocity;
    private double maxAcceleration;

    private double currentVelocity;
    private double currentAcceleration;

    private double trajectoryLength;
    private double trajectoryDirection;

    public TrajectoryGenerator(double distance, double maxVelocity, double maxAcceleration)
    {
        this.trajectoryDirection = Math.signum(distance);
        this.trajectoryLength = Math.abs(distance);
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    public void calculatePositionalDerivatives(ElapsedTime currentTime) {
        if (velocityIfConstantAcceleration(currentTime) < velocityIfCruising()
                && velocityIfConstantAcceleration(currentTime)
                < velocityIfConstantDeceleration(currentTime)) {
            currentVelocity = velocityIfConstantAcceleration(currentTime);
            currentAcceleration = maxAcceleration;
        } else if (velocityIfConstantDeceleration(currentTime) < velocityIfCruising()
                && velocityIfConstantDeceleration(currentTime)
                < velocityIfConstantAcceleration(currentTime)) {
            currentVelocity = velocityIfConstantDeceleration(currentTime);
            currentAcceleration = -maxAcceleration;
        } else {
            currentVelocity = maxVelocity;
            currentAcceleration = 0;
        }
    }

    public boolean isDirectionPositive(double distance) {
        return distance > 0;
    }

    private double velocityIfConstantAcceleration(ElapsedTime currentTime) {
        return maxAcceleration * currentTime.seconds();
    }

    private double velocityIfCruising(){
        return maxVelocity;
    }

    private double velocityIfConstantDeceleration(ElapsedTime currentTime) {
        double finalVelocity  = Math.sqrt(2 * maxAcceleration * trajectoryLength);
        return  finalVelocity - maxAcceleration * currentTime.seconds();
    }

    public double getDirection() {
        return trajectoryDirection;
    }

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public double getCurrentAcceleration() {
        return currentAcceleration;
    }
}
