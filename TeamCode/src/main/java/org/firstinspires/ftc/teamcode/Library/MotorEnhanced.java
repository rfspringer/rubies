package org.firstinspires.ftc.teamcode.Library;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class MotorEnhanced {
    public static void setRunMode(DcMotor[] dcMotors, DcMotor.RunMode runMode) {
        for (DcMotor motor : dcMotors) {
            motor.setMode(runMode);
        }
    }

    public static void setDirection(DcMotor[] dcMotors, DcMotor.Direction direction){
        for (DcMotor motor : dcMotors) {
            motor.setDirection(direction);
        }
    }

    public static void setPower(DcMotor[] dcMotors, double power) {
        power = Range.clip(power, -1, 1);
        for (DcMotor motor : dcMotors) {
            motor.setPower(power);
        }
    }

    public static void setPower(DcMotor motor, double power) {
        motor.setPower(power);
    }

    public static void setTargetPosition(DcMotor[] dcMotors, int targetPosition) {
        for (DcMotor motor : dcMotors) {
            motor.setTargetPosition(targetPosition);
        }
    }
}
