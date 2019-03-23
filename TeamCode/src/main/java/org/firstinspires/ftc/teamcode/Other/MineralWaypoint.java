package org.firstinspires.ftc.teamcode.Other;

public class MineralWaypoint {
    private double dTime;    //seconds
    private double linearPosition;  //inches
    private double angle;   //radians
    private double extensionPower;  //0-1
    private double angularPower;    //0-1
    private double angularVelocity; //inches per second
    private double powerArray;

    public MineralWaypoint(double angle, double dTime, double length, double extensionPower, double angularPower, double angularVelocity) {
        this.dTime = dTime;
        this.linearPosition = length;
        this.angle = angle;
        this.extensionPower = extensionPower;
        this.angularPower = angularPower;
        this.angularVelocity = angularVelocity;
        double[] powerArray = {extensionPower, angularPower};
    }

    public double getExtensionPower() {
        return extensionPower;
    }

    public double getLinearPosition() {
        return linearPosition;
    }

    public double getdTime() {
        return dTime;
    }
}
