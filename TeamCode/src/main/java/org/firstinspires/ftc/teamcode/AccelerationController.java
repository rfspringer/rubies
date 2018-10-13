package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AccelerationController {
    private ElapsedTime timer = new ElapsedTime();
    public double currentPower = 0;
    public double lastPower = 0;
    public double currentTime = 0;
    public double lastTime = 0;
    public double dPower;
    public double dTime;
    public boolean hasStartedTimer = false;

    private double maxAcceleration;

    public AccelerationController(double maxAccelerationOfPowers) {
        this.maxAcceleration = maxAccelerationOfPowers;
    }

    public void run(double targetPower, DcMotor[] motors) {
        updateTime();
        updateDifferentials(targetPower);
        currentPower = calculatePower();
        MotorEnhanced.setPowers(motors, currentPower);
        updateLastTimeAndPower();
    }

    public void run(double targetPower, DcMotor motor) {
        updateTime();
        updateDifferentials(targetPower);
        currentPower = calculatePower();
        motor.setPower(currentPower);
        updateLastTimeAndPower();
    }

    private void updateTime() {
        if (!hasStartedTimer) {
            timer.reset();
            hasStartedTimer = true;
        }
        currentTime = timer.seconds();
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
        double direction;
        if (dPower > 0) {
            direction = 1;
        } else if (dPower < 0) {
            direction = -1;
        } else {
            direction = 0;
        }
//        double direction = -Math.signum(dPower);
        dPower = Math.min(maxAcceleration * dTime,  dPower);
        return lastPower + direction * dPower;
    }
}
