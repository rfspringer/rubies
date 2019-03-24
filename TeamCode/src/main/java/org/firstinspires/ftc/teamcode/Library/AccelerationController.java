package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AccelerationController {
    private ElapsedTime timer = new ElapsedTime();
    private double currentPower = 0;
    private double lastPower = 0;
    private double currentTime = 0;
    private double lastTime = 0;
    private double dPower;
    private double dTime;
    private boolean hasStartedTimer = false;

    private double maxAcceleration;

    public AccelerationController(double maxAccelerationOfPowers) {
        this.maxAcceleration = maxAccelerationOfPowers;
    }

    public void run(double targetPower, DcMotor motor) {
        DcMotor[] motorArray = {motor};
        run(targetPower, motorArray);
    }

    public void run(double targetPower, DcMotor[] motors) {
        updateTime();
        updateDifferentials(targetPower);
        currentPower = calculatePower();
        MotorEnhanced.setPower(motors, currentPower);
        updateLastTimeAndPower();
    }

    private void updateTime() {
        if (!hasStartedTimer) {
            timer.reset();
            hasStartedTimer = true;
        }
        currentTime = timer.seconds();
    }

    public double accelerate(double dTime, double previousPower, double targetPower) {
        this.dTime = dTime;
        return calculatePower();
    }

    /**
     * Updates values for power, time variables of feedback controller
     * Must occur every time through loop
     */
    private void updateDifferentials(double targetPower){
        dTime = currentTime - lastTime;
        dPower = targetPower - lastPower;
    }

    private void updateLastTimeAndPower() {
        lastTime = currentTime;
        lastPower = currentPower;
    }

    private double calculatePower() {
        double direction = Math.signum(dPower);
        dPower = Math.min(maxAcceleration * dTime,  Math.abs(dPower));
        return lastPower + direction * dPower;
    }
}
