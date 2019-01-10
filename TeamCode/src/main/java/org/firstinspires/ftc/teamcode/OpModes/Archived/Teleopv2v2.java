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

package org.firstinspires.ftc.teamcode.OpModes.Archived;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareMaps.Archived.Robotv2;
import org.firstinspires.ftc.teamcode.Library.AccelerationController;
import org.firstinspires.ftc.teamcode.Library.GamepadEnhanced;


@TeleOp(name="Teleopv2v1 Meet 3", group="Iterative Opmode")
//@Disabled
public class Teleopv2v2 extends OpMode {
    private Robotv2 robot = Robotv2.getInstance();
    private ElapsedTime runtime = new ElapsedTime();
    private GamepadEnhanced gamepadA = new GamepadEnhanced();
    private AccelerationController leftAccelerationController = new AccelerationController(6.0);
    private AccelerationController rightAccelerationController = new AccelerationController(6.0);
    private AccelerationController liftAccelerationController = new AccelerationController(3.0);

    private double leftPower;
    private double rightPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
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

        controlDrivetrain();
        controlLift();
        controlArm();
        controlIntake();

        telemetry.addData("Motors", "left (%.2f), right (%.2f)",
                robot.drive.getLeftMotors()[0].getPower(), robot.drive.getRightMotors()[0].getPower());
        telemetry.addData("Encoders", "left(%d) right (%d)",
                robot.drive.getLeftEncoderCounts(), robot.drive.getRightEncoderCounts());
    }

    private void controlArm() {
        if (gamepadA.left_bumper) {
            robot.mineral.mineralIntake.setScaledPower(0);
            robot.mineral.mineralArm.setPower(1);
        } else if (gamepadA.right_bumper) {
            robot.mineral.mineralIntake.setScaledPower(0);
            robot.mineral.mineralArm.setPower(-1);
        } else {
            robot.mineral.mineralArm.setPower(0);
        }
    }

    private void controlIntake() {
        if (gamepadA.getAxisAsButton(GamepadEnhanced.AXIS.AXIS_LEFT_TRIGGER)) {
            robot.mineral.mineralIntake.setScaledPower(-1);
        } else if (gamepadA.getAxisAsButton(GamepadEnhanced.AXIS.AXIS_RIGHT_TRIGGER)) {
            robot.mineral.mineralIntake.setScaledPower(1);
        } else if (gamepadA.getAxisAsButton(GamepadEnhanced.AXIS.AXIS_LEFT_TRIGGER) && gamepadA.getAxisAsButton(GamepadEnhanced.AXIS.AXIS_RIGHT_TRIGGER)){
            robot.mineral.mineralIntake.setScaledPower(0);
        }
    }

    private void controlLift() {
        if (gamepadA.dpad_up){
            liftAccelerationController.run(1, robot.lift.getMotor());
        } else if (gamepadA.dpad_down) {
            liftAccelerationController.run(-1, robot.lift.getMotor());
        } else {
            liftAccelerationController.run(0, robot.lift.getMotor());
        }
    }

    private void controlDrivetrain() {
        calculateMotorPowers();
        if (gamepadA.getAxisAsButton(GamepadEnhanced.AXIS.AXIS_LEFT_TRIGGER)) {
            leftAccelerationController.run(0.4 * leftPower, robot.drive.getLeftMotors());
            rightAccelerationController.run(0.4 *rightPower, robot.drive.getRightMotors());
        } else {
            leftAccelerationController.run(leftPower, robot.drive.getLeftMotors());
            rightAccelerationController.run(rightPower, robot.drive.getRightMotors());
        }
    }

    private void calculateMotorPowers() {
        leftPower    = -0.8 * gamepadA.left_stick_y;
        rightPower   = -0.8 * gamepadA.right_stick_y;
    }

    @Override
    public void stop() {
    }
}
