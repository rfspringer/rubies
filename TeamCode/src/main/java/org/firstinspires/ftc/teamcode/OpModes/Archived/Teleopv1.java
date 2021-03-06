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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareMaps.Archived.Robotv2;
import org.firstinspires.ftc.teamcode.Library.AccelerationController;
import org.firstinspires.ftc.teamcode.Library.GamepadEnhanced;
import org.firstinspires.ftc.teamcode.Library.MotorEnhanced;


@TeleOp(name="Teleopv2v1 - No MineralParent", group="Iterative Opmode")
@Disabled
public class Teleopv1 extends OpMode
{
    private Robotv2 robot = Robotv2.getInstance();
    private ElapsedTime runtime = new ElapsedTime();
    private GamepadEnhanced gamepadA = new GamepadEnhanced();
    private AccelerationController leftAccelerationController = new AccelerationController(1.0);
    private AccelerationController rightAccelerationController = new AccelerationController(1.0);
    private AccelerationController liftAccelerationController = new AccelerationController(1.5);

    private double leftPower;
    private double rightPower;

    /*
     * Code to runAction ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        MotorEnhanced.setRunMode(robot.drive.getAllMotors(), DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to runAction REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        gamepadA.update(gamepad1);
        setDriveMotorPowers();
        moveLift();
        telemetry.addData("Motors", "left (%.2f), right (%.2f)",
                robot.drive.getLeftMotors()[0].getPower(), robot.drive.getRightMotors()[0].getPower());
        telemetry.addData("Encoders", "left(%d) right (%d)",
                robot.drive.getLeftEncoderCounts(), robot.drive.getRightEncoderCounts());
    }

    private void moveLift() {
        if (gamepadA.dpad_up){
            liftAccelerationController.run(1, robot.lift.getMotor());
        } else if (gamepadA.dpad_down) {
            liftAccelerationController.run(-1, robot.lift.getMotor());
        } else {
            liftAccelerationController.run(0, robot.lift.getMotor());
        }
    }

    private void setDriveMotorPowers() {
        calculateMotorPowers();
        if (gamepadA.left_bumper) {
            robot.drive.setPowers(leftPower, rightPower);
        } else {
            robot.drive.setPowers(leftPower, rightPower);
//            leftAccelerationController.runAction(leftPower, robot.drive.getLeftMotors());
//            rightAccelerationController.runAction(rightPower, robot.drive.getRightMotors());
        }
    }

    private void calculateMotorPowers() {
        leftPower    = -0.5 * gamepadA.left_stick_y;
        rightPower   = -0.5 * gamepadA.right_stick_y;
    }

    @Override
    public void stop() {
    }
}
