package org.firstinspires.ftc.teamcode;

public class OldTrajectoryGenerator {
    private double trajectoryLength;

    public OldTrajectoryGenerator(double distance) {
        this.trajectoryLength = distance;
    }

    private double distanceToAccelerateAndDecelerate (double acceleration, double maxVelocity) {
        // Rearranged kinematic equation v^2 = v_0^2 + 2ax
        return maxVelocity * maxVelocity / acceleration;
    }

    private double getCruisingDistance(double acceleration, double maxVelocity) {
        return trajectoryLength - distanceToAccelerateAndDecelerate(acceleration, maxVelocity);
    }

    private boolean trajectoryIsLongerThanAccelerationAndDeceleration
            (double distanceToAccelerateAndDecelerate, double trajectoryLength) {
        return  trajectoryLength > distanceToAccelerateAndDecelerate;
    }
}
