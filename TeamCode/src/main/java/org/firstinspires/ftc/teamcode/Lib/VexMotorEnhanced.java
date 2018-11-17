package org.firstinspires.ftc.teamcode.Lib;

import com.qualcomm.robotcore.hardware.CRServo;

public class VexMotorEnhanced {
    private static double MAX_POWER = 0.4;
    private static double MAX_POWER_RECIPROCAL = 1/MAX_POWER;

    /**
     * This function adjusts inputted power to be on the standard -1 thru 1 scale
     * It then converts this to a raw power and sets the vex motor power accordingly
     * @param power power for servo from -1.0 to 1.0
     */
    public static void setScaledPower(CRServo servo, double power) {
        servo.setPower(scalePower(power));
    }

    public static double getScaledPower(CRServo servo) {
        return unscalePower(servo.getPower());
    }

    private static double scalePower(double rawPower) {
        return rawPower * MAX_POWER;
    }

    private static double unscalePower(double scaledPower) {
        return scaledPower * MAX_POWER_RECIPROCAL;
    }
}
