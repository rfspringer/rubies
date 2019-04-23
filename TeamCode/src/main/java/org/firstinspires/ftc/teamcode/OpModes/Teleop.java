/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareMaps.Robot;
import org.firstinspires.ftc.teamcode.Library.AccelerationController;
import org.firstinspires.ftc.teamcode.Library.GamepadEnhanced;
import org.firstinspires.ftc.teamcode.Library.RubiesLinearOpMode;

@TeleOp(name="Teleop", group="teleop")
public class Teleop extends RubiesLinearOpMode {
    private Robot robot = Robot.getInstance();
    private ElapsedTime runtime = new ElapsedTime();
    private GamepadEnhanced gamepadA = new GamepadEnhanced();
    private GamepadEnhanced gamepadB = new GamepadEnhanced();
    private AccelerationController liftAccelerationController = new AccelerationController(3.0);
    private AccelerationController pivotAccelerationController = new AccelerationController(0.75);

    private double magnitudeMultiplier;

    private double HIGH_MAGNITUDE_MULTIPLIER = 0.8;
    private double LOW_MAGNITUDE_MULTIPLIER = 0.5;
    private double X_AXIS_THRESHOLD_FOR_TURNING = 0.7;
    private double HEADING_ERROR_SCALAR = 0.4;
    private double ARM_POWER_SCALAR = 0.6;
    private double INTAKE_POWER = 1;
    private double OUTTAKE_POWER = -1;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        robot.drive.setInAutonomous(false);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset();
        robot.claim.deploy();   //keeps claim out of the way of wires

        while (isStarted()) {
            if (isStopRequested()) {
                break;
            }

            gamepadA.update(gamepad1);
            gamepadB.update(gamepad2);

            controlDrive();
            controlArm();
            controlMineralDoor();
            controlIntake();
            controlExtension();
            controlLift();
            controlPin();

            telemetry.addData("Pivot Position", robot.mineral.getAngle());
            telemetry.addData("Acceleration from grav", robot.mineral.getAngularAccelerationFromGravity());
            telemetry.addData("Extension length", robot.mineral.getExtensionLength());
            telemetry.update();
        }
        robot.logger.closeFile();
    }

    private void controlDrive() {
        if (!gamepadA.left_bumper) {
            magnitudeMultiplier = HIGH_MAGNITUDE_MULTIPLIER;
        } else {
            magnitudeMultiplier = LOW_MAGNITUDE_MULTIPLIER;
        }
        robot.drive.setPowers(magnitudeMultiplier * gamepadA.getMagnitude(GamepadEnhanced.STICK.RIGHT_STICK),
                gamepadA.left_stick_x, -gamepadA.left_stick_y, getHeadingCorrection());
    }

    private double getHeadingCorrection() {
        if (Math.abs(gamepadA.right_stick_x) < X_AXIS_THRESHOLD_FOR_TURNING) {
            return 0;
        } else {
            return HEADING_ERROR_SCALAR * -gamepadA.right_stick_x;
        }
    }

    private void controlArm() {
        robot.mineral.setArmPower(-ARM_POWER_SCALAR * gamepadB.left_stick_y);
    }

    private void controlIntake() {
        if (gamepadB.getAxisAsButton(GamepadEnhanced.AXIS.AXIS_LEFT_TRIGGER)) {
            robot.mineral.intake();
        } else if (gamepadB.getAxisAsButton(GamepadEnhanced.AXIS.AXIS_RIGHT_TRIGGER)) {
            robot.mineral.outtake();
        } else if (gamepadB.getAxisAsButton(GamepadEnhanced.AXIS.AXIS_LEFT_TRIGGER) && gamepadA.getAxisAsButton(GamepadEnhanced.AXIS.AXIS_RIGHT_TRIGGER)){
            robot.mineral.setIntakeScaledPower(0);
        }
    }

    private void controlMineralDoor() {
        if (gamepadB.b) {
            robot.mineral.storeMinerals();
        } else if (gamepadB.a) {
            robot.mineral.stopIntake();
            robot.mineral.dumpMinerals();
        }
    }

    private void controlExtension() {
        robot.mineral.setExtensionPower(gamepadB.right_stick_y);
    }

    private void controlLift() {
        if (gamepadB.dpad_up){
            liftAccelerationController.run(-1, robot.lift.getMotor());
        } else if (gamepadB.dpad_down) {
            liftAccelerationController.run(1, robot.lift.getMotor());
        } else if (gamepadB.dpad_right) {
            robot.lift.setTargetPosition(0);
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lift.setPower(0.9);
        } else {
            liftAccelerationController.run(0, robot.lift.getMotor());
        }
    }

    private void controlPin() {
        if (gamepadA.y) {
            robot.lift.removePin();
        } else {
            robot.lift.stopPin();
        }
    }
}
