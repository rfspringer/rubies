package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class MotorEnhanced {
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

    public static void setPowers(DcMotor[] dcMotors, double power) {
        power = Range.clip(power, -1, 1);
        for (DcMotor motor : dcMotors) {
            motor.setPower(power);
        }
    }
}
