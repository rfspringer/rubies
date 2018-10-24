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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import org.firstinspires.ftc.teamcode.Lib.MotorEnhanced;
import org.firstinspires.ftc.teamcode.Lib.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.Lib.TrajectoryGenerator;

/**
 * This class stores all objects on our robot's drivetrain
 * It also includes functionality specific to our drive base
 */
public class Drive
{
    private static final Drive instance = new Drive();
    /* Public OpMode members. */
    private DcMotor leftDrive1 = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive1 = null;
    private DcMotor rightDrive2 = null;

    private DcMotor[] allMotors;
    private DcMotor[] leftMotors;
    private DcMotor[] rightMotors;

    private boolean reverseDirection = false;

    /* local OpMode members. */
    private HardwareMap hwMap =  null;

    private double MAX_VEL;
    private double MAX_ACCEL = ;

    /* Constructor */
    private Drive(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        initializeDriveMotors();
        initializeMotorArrays();
        setMotorDirections();
        setPowers(0, 0);
        MotorEnhanced.setRunMode(allMotors, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorEnhanced.setRunMode(allMotors, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initializeDriveMotors(){
        leftDrive1 = hwMap.get(DcMotor.class, "left_drive_1");
        leftDrive2 = hwMap.get(DcMotor.class, "left_drive_2");
        rightDrive1 = hwMap.get(DcMotor.class, "right_drive_1");
        rightDrive2 = hwMap.get(DcMotor.class, "right_drive_2");
    }

    private void initializeMotorArrays() {
        DcMotor[] allMotors = {leftDrive1, leftDrive2, rightDrive1, rightDrive2};
        DcMotor[] leftMotors = {leftDrive1, leftDrive2};
        DcMotor[] rightMotors = {rightDrive1, rightDrive2};
        this.allMotors = allMotors;
        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;
    }

    public void setPowers(double leftPower, double rightPower){
        MotorEnhanced.setPower(leftMotors, leftPower);
        MotorEnhanced.setPower(rightMotors, rightPower);
    }

    public void followTrajectory(double distance, double heading) {
        TrajectoryGenerator trajectory = new TrajectoryGenerator(distance, MAX_VEL, MAX_ACCEL);
        TrajectoryFollower trajectoryFollower = new TrajectoryFollower(allMotors, trajectory, kV, kA);
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

    public int getRightEncoderCounts(){
        double counts = (rightDrive1.getCurrentPosition() + rightDrive2.getCurrentPosition())/2;
        return (int) counts;
    }

    public int getLeftEncoderCounts(){
        double counts = (leftDrive1.getCurrentPosition() + leftDrive2.getCurrentPosition())/2;
        return (int) counts;
    }

    public DcMotor[] getAllMotors() {
        return allMotors;
    }

    public DcMotor[] getLeftMotors() {
        return leftMotors;
    }

    public DcMotor[] getRightMotors() {
        return rightMotors;
    }

    public static Drive getInstance() {
        return instance;
    }

}

