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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HWMaps.Robot;
import org.firstinspires.ftc.teamcode.Lib.FTCLogger;
import org.firstinspires.ftc.teamcode.Lib.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.Lib.TrajectoryGenerator;


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

@TeleOp(name="kA Test", group="tests")
@Disabled
public class kATest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime accelerationTimer = new ElapsedTime();

    private Robot robot = Robot.getInstance();
    private FTCLogger logger = new FTCLogger("kATest");
    private double MAX_VELOCITY = robot.drive.getMaxVelocity();
    private double MAX_ACCELERATION = robot.drive.getMaxAccel();
    private double kV = robot.drive.getkV();
    private double kA = 0.005;

    @Override
    public void runOpMode() {
        //NOT WORKING
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("This program will attempt to run a 3 foot trajectory. The acceleration constant is adjustable via the gamepad in init. Encoder values will be logged to a file which can be viewed in Excel", "Go RUBIES!");
        telemetry.addData("Instruction", "Press A to begin adjusting kA");
        telemetry.update();

        adjustkA();
        logger.writeLine("Max Velocity", "Max Acceleration", "kV", "kA");
        waitForStart();
        runtime.reset();
        logger.writeLine(MAX_VELOCITY, MAX_ACCELERATION, kV, kA);
        logger.writeLine("Motor Power", "Inches travelled");

        while (opModeIsActive()) {
            this.followTrajectory(36, 0, MAX_VELOCITY, MAX_ACCELERATION, false);

            telemetry.addData("Read distance", "%f", robot.drive.convertEncoderCountsToInches(robot.drive.getAverageEncoderCounts()));
            logger.writeLine(robot.drive.getLeftMotors()[0].getPower(), robot.drive.convertEncoderCountsToInches(robot.drive.getAverageEncoderCounts()));
        }
        logger.closeFile();
    }

    private void followTrajectory(double distanceInInches, double heading, double maxVel, double maxAccel, boolean usesFeedback) {
//        TrajectoryGenerator trajectory = new TrajectoryGenerator(distanceInInches, maxVel, maxVel);
//        TrajectoryFollower trajectoryFollower = new TrajectoryFollower(robot.drive.getAllMotors(), trajectory, kV, kA, usesFeedback);
//        trajectoryFollower.run();
    }

    private void adjustkA(){
        while (!isStarted()){
            if (gamepad1.dpad_up && accelerationTimer.milliseconds() > 500){
                //If the dpad is pushed up for more than 1/2 second, add a second of delay
                kA = kA + 0.005;
                accelerationTimer.reset();
            } else if (gamepad1.dpad_down && accelerationTimer.milliseconds() > 500) {
                //If the dpad is pushed down for more than 1/2 second, add a second of delay
                kA = kA - 0.005;
                accelerationTimer.reset();
            }

            //Display telemetry for the current delay time
            telemetry.addData("Current Acceleration", "%f" + MAX_ACCELERATION);
            telemetry.update();
        }
    }
}
