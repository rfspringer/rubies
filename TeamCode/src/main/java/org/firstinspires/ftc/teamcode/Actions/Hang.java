package org.firstinspires.ftc.teamcode.Actions;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Hang extends Action {
    @Override
    public void init() {
        robot.lift.getMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void run() {
        if (!actionIsComplete) {
            robot.lift.getMotor().setTargetPosition(0);
        }
    }
}
