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

import org.firstinspires.ftc.teamcode.HWMaps.Robotv2;
import org.firstinspires.ftc.teamcode.Lib.FTCLogger;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robotv2 Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drivev2 Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Velocity Test", group="Tests")
//@Disabled
public class VelocityTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime pathTime = new ElapsedTime();
    private Robotv2 robot = Robotv2.getInstance();
    private FTCLogger logger = new FTCLogger();

    private boolean hasSetEncoderValueAt2Seconds = false;
    private boolean hasCalculatedEncoderDiff = false;

    private double encoderValueAt2Seconds;
    private double encoderDiff;
    private double inchesPerSecond;
    private double powerOfMaxVel = 0.8;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Run, the encoders per sec value will give you the number of encoder counts in the last second of a 3 second runAction at 0.8 power", "Have fun :)");
        telemetry.update();

        logger.writeLine("Inches travelled in final second");
        waitForStart();
        pathTime.reset();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                pathTime.reset();
            }

            if (pathTime.seconds() < 3) {
                robot.drive.setPowers(powerOfMaxVel, powerOfMaxVel);
                setEncoderValueAt2SecondsIfApplicable();
            } else {
                robot.drive.setPowers(0,0);
                if (!hasCalculatedEncoderDiff) {
                    calculateEncoderDiff();
                    logger.writeLine(inchesPerSecond);
                    hasCalculatedEncoderDiff = true;
                }
            }
            addTelemetry();
            telemetry.update();
        }
        logger.closeFile();
    }

    private void setEncoderValueAt2SecondsIfApplicable() {
        if (pathTime.seconds() > 2 && !hasSetEncoderValueAt2Seconds){
            encoderValueAt2Seconds = robot.drive.getAverageEncoderCounts();
            hasSetEncoderValueAt2Seconds = true;
        }
    }

    private void calculateEncoderDiff (){
        encoderDiff = robot.drive.getAverageEncoderCounts() - encoderValueAt2Seconds;
        inchesPerSecond = robot.drive.convertEncoderCountsToInches(encoderDiff);
    }

    private void addTelemetry() {
        telemetry.addData("Inch per sec", inchesPerSecond);
        telemetry.addData("Encoders per sec", encoderDiff);
        telemetry.addData("Left", robot.drive.getLeftEncoderCounts());
        telemetry.addData("Right", robot.drive.getRightEncoderCounts());
        telemetry.addData("Instructions", "Press A to repeat and collect additional data");
    }
}
