package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

abstract public class FeedbackController {
    private ElapsedTime timer = new ElapsedTime();
    protected double currentPower = 0;
    protected double lastPower = 0;
    protected double currentTime = 0;
    protected double lastTime = 0;
    protected double dPower;
    protected double dTime;

    public void startController() {
        timer.reset();
    }

    /**
     * Updates values for power, time variables of feedback controller
     * Must occur every time through loop
     */
    public void updatePowerAndTime(double targetPower){
        currentTime = timer.seconds();
        dTime = currentTime - lastTime;
        dPower = targetPower - lastPower;
    }

    public void updatePowerAndTimeAfterCalculations() {
        lastTime = currentTime;
        lastPower = currentPower;
    }
//
//    public double getCurrentPower() {
//        return currentPower;
//    }
//
//    public double getLastPower() {
//        return lastPower;
//    }
//
//    public double getCurrentTime() {
//        return currentTime;
//    }
//
//    public double getLastTime() {
//        return lastTime;
//    }
//
//    public double getdPower() {
//        return dPower;
//    }
//
//    public double getdTime() {
//        return dTime;
//    }
//
//    public void setCurrentPower(double currentPower) {
//        this.currentPower = currentPower;
//    }
//
//    public void setdPower(double dPower) {
//        this.dPower = dPower;
//    }
//
//    public void setdTime(double dTime) {
//        this.dTime = dTime;
//    }
}
