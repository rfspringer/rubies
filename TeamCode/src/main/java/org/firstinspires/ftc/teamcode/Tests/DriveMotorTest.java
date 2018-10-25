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

package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HWMaps.Robot;
import org.firstinspires.ftc.teamcode.Lib.FTCLogger;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Motor Test", group="Tests")
public class DriveMotorTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private boolean leftMotor0 = false;
    private boolean leftMotor1 = false;
    private boolean rightMotor0 = false;
    private boolean rightMotor1 = false;

    private Robot robot = Robot.getInstance();
    private FTCLogger logger = new FTCLogger("Motor Test");

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        adjustMotors();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (runtime.seconds() < 3){
                if (leftMotor1)
                    robot.drive.getLeftMotors()[0].setPower(0.8);
                if (leftMotor0)
                    robot.drive.getLeftMotors()[1].setPower(0.8);
                if (rightMotor1)
                    robot.drive.getRightMotors()[0].setPower(0.8);
                if (rightMotor0)
                    robot.drive.getRightMotors()[1].setPower(0.8);
            } else {
                robot.drive.setPowers(0,0);
            }

            addTelemetry();
            telemetry.update();
        }
        logger.closeFile();
    }

    void adjustMotors(){
        while (!isStarted()){
            if (gamepad1.x)
                leftMotor1 = true;
             else if (gamepad1.y)
                 leftMotor0 = true;
             else if (gamepad1.a)
                 rightMotor1 = true;
             else if (gamepad1.b)
                 rightMotor0 = true;

            telemetry.addData("Left Motor 0 (x)", leftMotor1);
            telemetry.addData("Left Motor 1 (y)", leftMotor0);
            telemetry.addData("Right Motor 0 (a)", rightMotor1);
            telemetry.addData("Right Motor 1 (b)", rightMotor0);
            telemetry.update();
        }
    }

    private void addTelemetry() {
        telemetry.addData("Left Motor 0", robot.drive.getLeftMotors()[0].getCurrentPosition());
        telemetry.addData("Left Motor 1", robot.drive.getLeftMotors()[1].getCurrentPosition());
        telemetry.addData("Right Motor 0", robot.drive.getRightMotors()[0].getCurrentPosition());
        telemetry.addData("Right Motor 1", robot.drive.getRightMotors()[1].getCurrentPosition());

        telemetry.addData("Left", robot.drive.getLeftEncoderCounts());
        telemetry.addData("Right", robot.drive.getRightEncoderCounts());
    }
}
