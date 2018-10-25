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

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;


@TeleOp(name="Pixy Test", group="Tests")
public class PixyTest extends LinearOpMode {
    private I2cDeviceSynch pixyCam;
    private byte[] pixyData;

    private double x;
    private double y;
    private double width;
    private double height;
    private double numObjects;

    @Override
    public void runOpMode() {
        pixyCam = hardwareMap.i2cDeviceSynch.get("pixy");

        waitForStart();

        while (opModeIsActive()) {
            pixyCam.engage();

            //read 5 (creg) bytes on the second register (ireg) of the Pixy
            pixyData = pixyCam.read(0x52, 5);

            addTelemetry();
            telemetry.update();
            sleep(500);
        }
    }

    private void query() {
        x = pixyData[1];
        y = pixyData[2];
        width = pixyData[3];
        height = pixyData[4];
        numObjects = pixyData[0];
    }

    private void addTelemetry() {
        telemetry.addData("0", 0xff & pixyData[0]);
        telemetry.addData("1", 0xff & pixyData[1]);
        telemetry.addData("2", 0xff & pixyData[2]);
        telemetry.addData("3", 0xff & pixyData[3]);
        telemetry.addData("4", 0xff & pixyData[4]);
        telemetry.addData("Length", pixyData.length);
    }
}
