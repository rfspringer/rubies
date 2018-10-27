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

package org.firstinspires.ftc.teamcode.HWMaps;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Lib.PIDController;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for our robot
 * This class stores functions that 3use a combination of subsystems on our robot
 */
public class Robot
{
    private static final Robot instance = new Robot();
    public Drive drive = Drive.getInstance();
    public Lift lift = Lift.getInstance();
    public Claim claim = Claim.getInstance();
    public Sensors sensors = Sensors.getInstance();

    /* Constructor */
    private Robot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        drive.init(hwMap);
        lift.init(hwMap);
        sensors.init(hwMap);
        claim.init(hwMap);
    }

    public void turnToHeading(double targetHeading) {
        while (Math.abs(sensors.integrateHeading(targetHeading - sensors.getHeading())) > 2.5) {
            sensors.updateIMU();
            double kP = 0.0065;
            double error = targetHeading - sensors.getHeading();
            double leftPower = PIDController.proportionalController(0, error, -kP);
            double rightPower = PIDController.proportionalController(0, error, kP);
            drive.setPowers(leftPower, rightPower);
        }
        drive.setPowers(0, 0);
    }

    public static Robot getInstance() {
        return instance;
    }
 }

