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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareMaps.Robot;
import org.firstinspires.ftc.teamcode.Library.AccelerationController;
import org.firstinspires.ftc.teamcode.Library.GamepadEnhanced;


@TeleOp(name="Teleop", group="teleop")
public class Teleop extends OpMode {
    private Robot robot = Robot.getInstance();
    private ElapsedTime runtime = new ElapsedTime();
    private GamepadEnhanced gamepadA = new GamepadEnhanced();
    private GamepadEnhanced gamepadB = new GamepadEnhanced();
    private AccelerationController liftAccelerationController = new AccelerationController(3.0);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.drive.setInAutonomous(false);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        gamepadA.update(gamepad1);
        gamepadB.update(gamepad2);

        controlDrive();
        controlArm();
        controlBucket();
        controlIntake();
        controlExtension();
        controlLift();
        controlPin();
    }

    private void controlDrive() {
        if (!gamepadA.left_bumper) {
            robot.drive.setPowers(gamepadA.getMagnitude(GamepadEnhanced.STICK.RIGHT_STICK),
                    gamepadA.left_stick_x, -gamepadA.left_stick_y, getHeadingCorrection());
        } else {
            robot.drive.setPowers(0.5 * gamepadA.getMagnitude(GamepadEnhanced.STICK.RIGHT_STICK),
                    gamepadA.left_stick_x, -gamepadA.left_stick_y, getHeadingCorrection());
        }
    }

    private double getHeadingCorrection() {
        if (Math.abs(gamepadA.right_stick_x) < 0.7) {
            return 0;
        } else {
            return -0.4 * gamepadA.right_stick_x;
        }
    }

    private void controlArm() {
        robot.mineral.setArmPower(-0.3 * gamepadB.left_stick_y);
    }

    private void controlIntake() {
        if (gamepadB.getAxisAsButton(GamepadEnhanced.AXIS.AXIS_LEFT_TRIGGER)) {
            robot.mineral.setIntakeScaledPower(1);
        } else if (gamepadB.getAxisAsButton(GamepadEnhanced.AXIS.AXIS_RIGHT_TRIGGER)) {
            robot.mineral.setIntakeScaledPower(-1);
        } else if (gamepadB.getAxisAsButton(GamepadEnhanced.AXIS.AXIS_LEFT_TRIGGER) && gamepadA.getAxisAsButton(GamepadEnhanced.AXIS.AXIS_RIGHT_TRIGGER)){
            robot.mineral.setIntakeScaledPower(0);
        }
    }

    private void controlBucket() {
        if (gamepadB.y) {
            robot.mineral.setToIntake();
        } else if (gamepadB.b) {
            robot.mineral.storeMinerals();
        } else if (gamepadB.a) {
            robot.mineral.dumpMinerals();
        }
    }

    private void controlExtension() {
        robot.mineral.setExtensionPower(gamepadB.right_stick_y);
    }

    private void controlLift() {
        if (gamepadB.dpad_up){
            liftAccelerationController.run(1, robot.lift.getMotor());
        } else if (gamepadB.dpad_down) {
            liftAccelerationController.run(-1, robot.lift.getMotor());
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


    @Override
    public void stop() {
    }
}
