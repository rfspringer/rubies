package org.firstinspires.ftc.teamcode.Lib;

public class PIDController {
    public static double PID(double initialPower, double p, double i, double d, double kP, double kI, double kD) {
        return initialPower + p * kP + i * kI + d * kD;
    }

    public static double proportionalController(double initialPower, double p, double kP) {
        return initialPower + p * kP;
    }
}
