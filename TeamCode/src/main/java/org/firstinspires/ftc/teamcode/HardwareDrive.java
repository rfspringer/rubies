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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This class stores all objects on our robot's drivetrain
 * It also includes functionality specific to our drive base
 */
public class HardwareDrive
{
    /* Public OpMode members. */
    private DcMotor  leftFrontDrive   = null;
    private DcMotor  leftBackDrive   = null;
    private DcMotor  rightFrontDrive  = null;
    private DcMotor  rightBackDrive  = null;

    private boolean reverseDirection = false;

    /* local OpMode members. */
    private HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareDrive(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        initializeDriveMotors();
        setMotorDirections();
        setPowers(0, 0);
        setRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initializeDriveMotors(){
        leftFrontDrive  = hwMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hwMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hwMap.get(DcMotor.class, "right_back_drive");
    }

    public void setRunModes(DcMotor.RunMode runMode){
        leftFrontDrive.setMode(runMode);
        leftBackDrive.setMode(runMode);
        rightFrontDrive.setMode(runMode);
        rightBackDrive.setMode(runMode);
    }


    public void setPowers(double leftPower, double rightPower){
        setLeftPower(leftPower);
        setRightPower(rightPower);
    }

    public void setLeftPower(double power){
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
    }

    public void setRightPower(double power){
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
    }

    public void reverseMotorDirections(boolean reverseDirection) {
        this.reverseDirection = reverseDirection;
        setMotorDirections();
    }

    public boolean isDirectionReversed() {
        return reverseDirection;
    }

    private void setMotorDirections(){
        if (reverseDirection){
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        } else {
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        }
    }

}

