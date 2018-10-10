package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class EnhancedMotor {
    public static void setRunModes(DcMotor[] dcMotors, DcMotor.RunMode runMode) {
        for (DcMotor motor : dcMotors) {
            motor.setMode(runMode);
        }
    }

    public static void setDirections(DcMotor[] dcMotors, DcMotor.Direction direction){
        for (DcMotor motor : dcMotors) {
            motor.setDirection(direction);
        }
    }
}
