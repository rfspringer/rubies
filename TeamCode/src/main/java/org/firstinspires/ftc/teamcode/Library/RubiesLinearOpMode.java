package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

    public void sleepFor(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        telemetry.addData("Status", "sleeping");
        telemetry.update();
    }
}
