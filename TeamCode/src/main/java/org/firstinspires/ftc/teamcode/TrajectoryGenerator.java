package org.firstinspires.ftc.teamcode;

public class TrajectoryGenerator {
    private Drive drive = Drive.getInstance();
    private double trajectoryLength;

    public TrajectoryGenerator (double distance) {
        this.trajectoryLength = distance;
    }

    private double distanceToAccelerateAndDecelerate (double maxAcceleration, double maxVelocity) {
        // Rearranged kinematic equation v^2 = v_0^2 + 2ax
        return Math.pow(maxVelocity, 2) / maxAcceleration;
    }

    private boolean trajectoryIsLongerThanAccelerationAndDeceleration
            (double distanceToAccelerateAndDecelerate, double trajectoryLength) {
        return  trajectoryLength > distanceToAccelerateAndDecelerate;
    }
}
