package org.firstinspires.ftc.teamcode.Library.Archived;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TrajectoryGeneratorByTime {
    // Units are inches and seconds
    private double maxVelocity;
    private double maxAcceleration;

    private double currentVelocity;
    private double currentAcceleration;

    private double trajectoryLength;
    private double trajectoryDirection;

    private double seconds;

    private enum TRAJECTORY_SEGMENT {
        ACCELERATION,
        CRUISING,
        DECELERATION
    }

    private TRAJECTORY_SEGMENT trajectorySegment;

    public TrajectoryGeneratorByTime(double distance, double maxVelocity, double maxAcceleration) {
        this.trajectoryDirection = Math.signum(distance);
        this.trajectoryLength = Math.abs(distance);
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    public void updateVelocityAndAcceleration(ElapsedTime currentTime) {
        calculatePositionalDerivatives(currentTime.seconds());
    }

    public void updateVelocityAndAcceleration(double currentTime) {
        calculatePositionalDerivatives(currentTime);
    }

    private void calculatePositionalDerivatives(double currentTime) {
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

    private TRAJECTORY_SEGMENT getTrajectorySegment (double currentTime) {
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

    private double velocityIfConstantAcceleration(double currentTime) {
        return maxAcceleration * currentTime;
    }


    private double velocityIfConstantDeceleration(double currentTime) {
        double finalVelocity  = Math.sqrt(2 * maxAcceleration * trajectoryLength);
        return  finalVelocity - maxAcceleration * currentTime;
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