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

package org.firstinspires.ftc.teamcode.HardwareMaps;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class stores all objects on our robot's drivetrain
 * It also includes functionality specific to our drive base
 */
public class MineralExtension {
    private static final MineralExtension instance = new MineralExtension();
    private DcMotor extension = null;
    private HardwareMap hwMap = null;

    private double RETRACTED_ARM_LENGTH;change
    private double METERS_PER_REVOLUTION;change
    private int COUNTS_PER_REVOLUTION = 288;
    private double MAX_VELOCITY;change

    /* Constructor */
    private MineralExtension(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        extension = hwMap.get(DcMotor.class, "extension");
        extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setDirection(DcMotorSimple.Direction.FORWARD);
        extension.setPower(0);
    }

    public void setPower(double power) {
        extension.setPower(power);
    }

    /**
     * @return the length of our mineral arm from pivot to tip in meters as a double
     */
    public double getCurrentLength() {
        //
        return RETRACTED_ARM_LENGTH + getNetExtensionDistance();
    }

    public double getPositionOfCenterOfGravity() {
        return getCurrentLength()/2; // it's close enough :)
    }

    public double calculateLengthFromIntegration(double previousLength, double extensionPower, double dTime) {
        double velocity = getVelocity(extensionPower);
        return previousLength + velocity * dTime;
    }

    private double getVelocity(double extensionPower) {
        return extensionPower * MAX_VELOCITY;
    }

    private double getNetExtensionDistance() {
        return getNetNumberOfRevolutions() * METERS_PER_REVOLUTION;
    }

    private double getNetNumberOfRevolutions() {
        return extension.getCurrentPosition() / COUNTS_PER_REVOLUTION;
    }

    public double getMomentofInertia(double length, double mass) {
        return 1/3 * mass * length * length;
    }

    public static MineralExtension getInstance(){
        return instance;
    }
}

