package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TrajectoryGenerator {
    // Units are inches and seconds
    private double maxVelocity;
    private double maxAcceleration;

    private double currentVelocity;
    private double currentAcceleration;

    private double trajectoryLength;

    public TrajectoryGenerator(double trajectoryLength, double maxVelocity, double maxAcceleration) {
        this.trajectoryLength = trajectoryLength;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    public void calculatePositionalDerivatives(ElapsedTime currentTime) {
        if (velocityIfConstantAcceleration(currentTime) < velocityIfCruising()
                && velocityIfConstantAcceleration(currentTime)
                < velocityIfConstantAcceleration(currentTime)) {
            currentVelocity = velocityIfConstantAcceleration(currentTime);
            currentAcceleration = maxAcceleration;
        } else if (velocityIfConstantDeceleration(currentTime) < velocityIfCruising()
                && velocityIfConstantDeceleration(currentTime) <
                velocityIfConstantAcceleration(currentTime)) {
            currentVelocity = velocityIfConstantAcceleration(currentTime);
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
        return  Math.sqrt(2 * maxAcceleration * trajectoryLength) - maxAcceleration
                * currentTime.seconds();
    }

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public double getCurrentAcceleration() {
        return currentAcceleration;
    }
}
