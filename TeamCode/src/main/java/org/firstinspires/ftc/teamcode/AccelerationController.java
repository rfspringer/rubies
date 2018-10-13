package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AccelerationController extends FeedbackController {
    private double maxAcceleration;

    public AccelerationController(double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
    }

    public void setPower(double targetPower, DcMotor[] motors) {
        updatePowerAndTime(targetPower);
        double power = calculatePower();
        MotorEnhanced.setPowers(motors, power);
        updatePowerAndTimeAfterCalculations();
    }

    private double calculatePower() {
        double direction = Math.signum(dPower);
        dPower = Math.min(maxAcceleration * dTime,  dPower);
        return lastPower + direction * dPower;
    }
}
