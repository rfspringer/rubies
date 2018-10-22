package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.IOError;
import java.io.IOException;

public class TrajectoryGenerator {
    // Units are inches and seconds
    private double maxVelocity;
    private double maxAcceleration;

    private double currentVelocity;
    private double currentAcceleration;

    private double trajectoryLength;

    public TrajectoryGenerator(double trajectoryLength, double maxVelocity, double maxAcceleration)
    {
        if (trajectoryLength < 0) {
            throw new java.lang.RuntimeException("Cannot have negative trajectory length");
        }
        this.trajectoryLength = trajectoryLength;
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

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public double getCurrentAcceleration() {
        return currentAcceleration;
    }
}
