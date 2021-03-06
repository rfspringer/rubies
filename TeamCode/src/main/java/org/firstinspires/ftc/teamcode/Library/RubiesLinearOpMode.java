package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class RubiesLinearOpMode extends LinearOpMode {
    @Override
    public synchronized void waitForStart() {
        while (!isStarted()) {
            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
            telemetry.addData("Status", "waiting for start");
            telemetry.update();
        }
    }

    protected void sleepFor(long milliseconds) {
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < milliseconds) {
            telemetry.addData("Status", "sleeping");
            telemetry.update();
        }
    }
}
