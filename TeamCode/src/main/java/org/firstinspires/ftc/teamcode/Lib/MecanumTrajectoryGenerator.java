package org.firstinspires.ftc.teamcode.Lib;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MecanumTrajectoryGenerator {
    // Units are inches and seconds
    private double maxVelocity;
    private double maxAcceleration;

    private double currentVelocity;
    private double currentAcceleration;

    private double x;
    private double y;

    private double trajectoryLength;
//    private double trajectoryDirection;

    public MecanumTrajectoryGenerator(double x, double y, double maxVelocity, double maxAcceleration) {
        this.x = x;
        this.y = y;
        this.trajectoryLength = Math.abs(getTrajectoryLength());
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

    private double getTrajectoryLength() {
        return Math.sqrt(x*x + y * y);
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

//    public double getDirection() {
//        return trajectoryDirection;
//    }

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public double getCurrentAcceleration() {
        return currentAcceleration;
    }
}
