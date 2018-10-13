package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FeedbackControllers {
    private ElapsedTime timer = new ElapsedTime();
    private double currentPower = 0;
    private double lastPower = 0;
    private double currentTime = 0;
    private double lastTime = 0;
    private double dPower;
    private double dTime;

    public void startController() {
        timer.reset();
    }

    /**
     * Updates values for power, time variables of feedback controller
     * Must occur every time through loop
     */
    public void updateValues(double currentPower){
        this.currentPower = currentPower;
        currentTime = timer.seconds();
        dTime = currentTime - lastTime;
        dPower = currentPower - lastPower;
    }

    private void updateValuesAfterLoop() {
        lastTime = currentTime;
        lastPower = currentPower;
    }

    public void controlAcceleration(DcMotor[] motors, double acceleration) {
        double direction = Math.signum(dPower);
        dPower = Math.min(acceleration * dTime,  dPower);
        double power = lastPower + direction * dPower;
        MotorEnhanced.setPowers(motors, power);
        updateValuesAfterLoop();
    }

}
