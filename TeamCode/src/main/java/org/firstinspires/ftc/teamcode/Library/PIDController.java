package org.firstinspires.ftc.teamcode.Library;

/**
 * This class contains static functions for PID controllers
 * Also contains PID controller variations
 */
public class PIDController {
    public static double PID(double initialPower, double p, double i, double d, double kP, double kI, double kD) {
        return initialPower + p * kP + i * kI + d * kD;
    }

    public static double pController(double initialPower, double p, double kP) {
        return PID(initialPower, p, 0, 0, kP, 0, 0);
    }
}
