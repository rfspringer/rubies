package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TrajectoryFollower {

    private enum State {
        ACCELERATING,
        CRUISING,
        DECELERATING
    }
    private State currentState = State.ACCELERATING;
    private Drive drive = Drive.getInstance();

    // All in milliseconds
    private double timeToChangeAcceleration;
    private double timeOfAcceleration;
    private double timeOfConstantVelocity;

    private double maxVelocity;
    private double maxAcceleration;
    private double maxJerk;

    private double kV;
    private double kA;

    public TrajectoryFollower() {

    }

    public void run(ElapsedTime currentTime) {
        if (currentTime.milliseconds() < timeOfAcceleration) {
            accelerate(currentTime);
        } else if (timeOfAcceleration <= currentTime.milliseconds() &&
                currentTime.milliseconds() < timeOfAcceleration + timeOfConstantVelocity) {
            cruise(currentTime);
        } else if (timeOfAcceleration + timeOfConstantVelocity < currentTime.milliseconds()) {
            decelerate(currentTime);
        }
    }

    private void accelerate(ElapsedTime currentTime) {

    }

    private void cruise(ElapsedTime currentTime) {
        setPowers();
    }

    private void decelerate(ElapsedTime currentTime) {

    }

    private void setPowers(double targetPower) {

    }

    public double getAcceleration() {
        double acceleration;
        if (currentState == State.ACCELERATING) {
            acceleration = maxAcceleration;
        } else if (currentState == State.DECELERATING) {
            acceleration = -maxAcceleration;
        } else {
            acceleration = 0.0;
        }

        return acceleration;
    }

    public double getVelocity(ElapsedTime currentTime) {
        double velocity;
        if (currentState == State.ACCELERATING) {
            velocity = ;
        } else if (currentState == State.DECELERATING) {
            velocity = ;
        } else {
            velocity = maxVelocity;
        }
    }

    private double

}
