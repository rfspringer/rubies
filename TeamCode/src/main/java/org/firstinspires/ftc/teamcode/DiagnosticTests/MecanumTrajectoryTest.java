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

package org.firstinspires.ftc.teamcode.DiagnosticTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareMaps.Robot;
import org.firstinspires.ftc.teamcode.Library.FTCLogger;
import org.firstinspires.ftc.teamcode.Library.MecanumTrajectoryFollower;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robotv3 Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleopv3 for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mecanum Trajectory Test", group="Tests")
//@Disabled
public class MecanumTrajectoryTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime accelerationTimer = new ElapsedTime();

    private Robot robot = Robot.getInstance();
    private FTCLogger logger = new FTCLogger("AccelerationTest");


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        MecanumTrajectoryFollower trajectory = robot.drive.initializeTrajectory(24, 0, 0, 24, false);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("This program will attempt to run a 2 footx 2 foot trajectory", "Go RUBIES!");
        telemetry.addData("Instruction", "Press A to begin adjusting acceleration");
        telemetry.addData("A", gamepad1.a);
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            trajectory.run();
            telemetry.addData("Powers", robot.drive.getAllMotors()[0].getPower());
//            telemetry.addData("Read distance", robot.drive.convertEncoderCountsToInches(robot.drive.getAverageEncoderCounts()));
            telemetry.update();
//            logger.writeLine(acceleration, robot.drive.convertEncoderCountsToInches(robot.drive.getAverageEncoderCounts()), robot.drive.getLeftMotors()[0].getPower(), robot.drive.getRightMotors()[0].getPower());
        }
        robot.logger.closeFile();
    }

}
