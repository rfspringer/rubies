package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareMaps.Robot;

import java.lang.annotation.ElementType;

public class TrajectoryGenerator {
    private MecanumEnhanced mecanumEnhanced = new MecanumEnhanced();
    // Units are inches and seconds
    private double maxVelocity;
    private double maxAcceleration;

    private double currentVelocity;
    private double currentAcceleration;

    private double trajectoryLength;
    private double currentPosition;
    private double totalTime;

    private double x = 0;
    private double y = 0;

    private enum TRAJECTORY_SEGMENT {
        ACCELERATION,
        CRUISING,
        DECELERATION
    }

    private TRAJECTORY_SEGMENT trajectorySegment;

    public TrajectoryGenerator(double trajectoryLength, double maxVelocity, double maxAcceleration) {
        this.trajectoryLength = trajectoryLength;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.totalTime = calculateTotalTime();
    }

    public TrajectoryGenerator(double x, double y, double velocityMagnitude, double maxAcceleration) {
        this.x = x;
        this.y = y;
        this.trajectoryLength = Math.abs(getTrajectoryLength(x, y));
        this.maxVelocity = mecanumEnhanced.getMaxVel(velocityMagnitude, x, y);
        this.maxAcceleration = maxAcceleration;
        this.totalTime = calculateTotalTime();
    }

    private double getTrajectoryLength(double x, double y) {
        return Math.sqrt(x*x + y*y);
    }

    public void calculatePositionalDerivatives(ElapsedTime currentTime) {
        trajectorySegment = getTrajectorySegment(currentTime);
        switch (trajectorySegment){
            case ACCELERATION:
                currentVelocity = velocityIfConstantAcceleration(currentTime);
                currentAcceleration = maxAcceleration;
                break;

            case DECELERATION:
                currentVelocity = velocityIfConstantDeceleration(currentTime);
                currentAcceleration = -maxAcceleration;
                break;

            case CRUISING:
                currentVelocity = maxVelocity;
                currentAcceleration = 0;
                break;
        }
    }

    private TRAJECTORY_SEGMENT getTrajectorySegment (ElapsedTime currentTime) {
        if (velocityIfConstantAcceleration(currentTime) < maxVelocity
                && velocityIfConstantAcceleration(currentTime)
                < velocityIfConstantDeceleration(currentTime)) {
            return TRAJECTORY_SEGMENT.ACCELERATION;
        } else if (velocityIfConstantDeceleration(currentTime) < maxVelocity
                && velocityIfConstantDeceleration(currentTime)
                < velocityIfConstantAcceleration(currentTime)) {
            return TRAJECTORY_SEGMENT.DECELERATION;
        } else {
            return TRAJECTORY_SEGMENT.CRUISING;
        }
    }

    private double getTrajectoryLength() {
        return trajectoryLength;
    }

    private double velocityIfConstantAcceleration(ElapsedTime currentTime) {
        return maxAcceleration * currentTime.seconds();
    }

    private double velocityIfConstantDeceleration(ElapsedTime currentTime) {
        double finalVelocity  = Math.sqrt(2 * maxAcceleration * trajectoryLength);
        return  finalVelocity - maxAcceleration * currentTime.seconds();
    }

    protected double calculateTotalTime() {
        return maxVelocity/maxAcceleration + trajectoryLength/maxVelocity;
    }

    public double getCurrentVelocity() {
        return currentVelocity;
    }

    public double getCurrentAcceleration() {
        return currentAcceleration;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}