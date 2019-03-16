package org.firstinspires.ftc.teamcode.Other;

public class MineralWaypoint {
    private double time;
    private double linearPosition;
    private double extensionPower;
    private double angularPower;
    private double angularVelocity;
    private double powerArray;

    public MineralWaypoint(double time, double linearPosition, double extensionPower, double angularPower, double angularVelocity) {
        this.time = time;
        this.linearPosition = linearPosition;
        this.extensionPower = extensionPower;
        this.angularPower = angularPower;
        this.angularVelocity = angularVelocity;
        double[] powerArray = {extensionPower, angularPower};
    }

    public double getLinearPosition() {
        return linearPosition;
    }

    public double getPowerArray() {
        return powerArray;
    }
}
