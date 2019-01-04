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
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Library.MecanumEnhanced;
import org.firstinspires.ftc.teamcode.Library.MotorEnhanced;

/**
 * This class stores all objects on our robot's drivetrain
 * It also includes functionality specific to our drive base
 */
public class MecanumDrive
{
    private static final MecanumDrive instance = new MecanumDrive();
    /* Public OpMode members. */
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    private DcMotor[] allMotors;
    private DcMotor[] leftMotors;
    private DcMotor[] rightMotors;

    private boolean reverseDirection = false;
    MecanumEnhanced mecanumEnhanced = new MecanumEnhanced();

    /* local OpMode members. */
    private HardwareMap hwMap =  null;

    /* Constructor */
    private MecanumDrive(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        initializeDriveMotors();
        initializeMotorArrays();
        setMotorDirections();
        setIndividualPowers(0, 0, 0, 0);
        MotorEnhanced.setRunMode(allMotors, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorEnhanced.setRunMode(allMotors, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initializeDriveMotors(){
        leftFront = hwMap.get(DcMotor.class, "left_front");
        leftBack = hwMap.get(DcMotor.class, "left_back");
        rightFront = hwMap.get(DcMotor.class, "right_front");
        rightBack = hwMap.get(DcMotor.class, "right_back");
    }

    private void initializeMotorArrays() {
        //note that arrays MUST be initialized in this order for Mecanum Enhanced to work
        DcMotor[] allMotors = {leftFront, leftBack, rightFront, rightBack};
        DcMotor[] leftMotors = {leftFront, leftBack};
        DcMotor[] rightMotors = {rightFront, rightBack};
        this.allMotors = allMotors;
        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;
    }

    public void setPowers(double magnitude, double x, double y, double heading) {
        double[] powers = mecanumEnhanced.calculatePowers(magnitude, x, y, heading);
        setIndividualPowers(powers[0], powers[1], powers[2], powers[3]);
    }

    public void setIndividualPowers(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower){
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
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
            MotorEnhanced.setDirection(leftMotors, Direction.FORWARD);
            MotorEnhanced.setDirection(rightMotors, Direction.REVERSE);
        } else {
            MotorEnhanced.setDirection(leftMotors, Direction.REVERSE);
            MotorEnhanced.setDirection(rightMotors, Direction.FORWARD);
        }
    }

    public void setInAutonomous(boolean inAutonomous) {
        mecanumEnhanced.setInAutonomous(inAutonomous);
    }

    public int getRightEncoderCounts(){
        double counts = (rightFront.getCurrentPosition() + rightBack.getCurrentPosition())/2;
        return (int) counts;
    }

    public int getLeftEncoderCounts(){
        double counts = (leftFront.getCurrentPosition() + leftBack.getCurrentPosition())/2;
        return (int) counts;
    }

    public int getAverageEncoderCounts(){
        double counts = (getLeftEncoderCounts() + getRightEncoderCounts())/2;
        return (int) counts;
    }

    public DcMotor[] getAllMotors() {
        return allMotors;
    }

    public static MecanumDrive getInstance() {
        return instance;
    }

}

