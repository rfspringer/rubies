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
import org.firstinspires.ftc.teamcode.Library.MecanumTrajectoryFollower;
import org.firstinspires.ftc.teamcode.Library.MotorEnhanced;
import org.firstinspires.ftc.teamcode.Library.PIDController;
import org.firstinspires.ftc.teamcode.Library.TensorFlow;
import org.firstinspires.ftc.teamcode.Library.TrajectoryGenerator;

/**
 * This class stores all objects on our robot's drivetrain
 * It also includes functionality specific to our drive base
 */
public class Drive
{
    private static final Drive instance = new Drive();
    MecanumEnhanced mecanumEnhanced = new MecanumEnhanced();

    /* Public OpMode members. */
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    private DcMotor[] allMotors;
    private DcMotor[] leftMotors;
    private DcMotor[] rightMotors;

    private boolean reverseDirection = false;
    private double kA = 0.0001;

    private double ENCODER_COUNTS_PER_ROTATION = 1120;
    private double DISTANCE_PER_REVOLUTION = 4 * Math.PI;

    private double MAX_FORWARD_VELOCITY = 25.6;
    private double MAX_STRAFE_VELOCITY = 20;
    private double MAX_ACCEL = 16;

    private double SAMPLING_DISTANCE_LEFT = -20;
    private double SAMPLING_DISTANCE_RIGHT = -20;
    private double SAMPLING_DISTANCE_CENTER = -14;

    private double BACKUP_DISTANCE_LEFT;
    private double BACKUP_DISTANCE_RIGHT;
    private double BACKUP_DISTANCE_CENTER;

    private double WALL_DISTANCE_LEFT;
    private double WALL_DISTANCE_RIGHT;
    private double WALL_DISTANCE_CENTER;

    private double DISTANCE_TO_WALL;
    private double DISTANCE_AWAY_FROM_WALL;

    private double DEPOT_WALL_HEADING;
    private double CRATER_WALL_HEADING;

    private double DEPOT_TO_DEPOT_DISTANCE;
    private double CRATER_TO_DEPOT_DISTANCE;

    private double PARKING_DISTANCE;


    private MecanumTrajectoryFollower unlatchAwayFromLander;
    private MecanumTrajectoryFollower unlatchParallelToLander;
    private MecanumTrajectoryFollower driveAwayFromLander;
    private MecanumTrajectoryFollower centerForSampling;
    private MecanumTrajectoryFollower driveAwayFromMarker;

    /* local OpMode members. */
    private HardwareMap hwMap =  null;

    /* Constructor */
    private Drive(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        initializeDriveMotors();
        initializeMotorArrays();
        initializeTrajectories();
        setIndividualPowers(0, 0, 0, 0);
        MotorEnhanced.setDirection(leftMotors, Direction.FORWARD);
        MotorEnhanced.setDirection(rightMotors, Direction.REVERSE);
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

    private void initializeTrajectories() {
        unlatchAwayFromLander = initializeTrajectory(-1.15, 0, 0);
        unlatchParallelToLander = initializeTrajectory(0, 3.75, 0);
        driveAwayFromLander = initializeTrajectory(-3, 0, 0);
        centerForSampling = initializeTrajectory(0, -2.5, 0);
        driveAwayFromMarker = initializeTrajectory(0, -5, 0);
    }

    public void runSamplingTrajectory (TensorFlow.GoldPosition goldPosition, double heading) {
        MecanumTrajectoryFollower trajectory = determineSamplingTrajectory(goldPosition, heading);
        trajectory.run();
    }

    private MecanumTrajectoryFollower determineSamplingTrajectory (TensorFlow.GoldPosition goldPosition, double heading) {
        if (goldPosition == TensorFlow.GoldPosition.LEFT) {
            return initializeTrajectory(0, SAMPLING_DISTANCE_LEFT, heading);
        } else if (goldPosition == TensorFlow.GoldPosition.RIGHT) {
            return initializeTrajectory(0, SAMPLING_DISTANCE_RIGHT, heading);
        } else {
            return initializeTrajectory(0, SAMPLING_DISTANCE_CENTER, heading);
        }
    }

    public void backupFromSampling(TensorFlow.GoldPosition goldPosition) {
       double  backupDistance = getBackupFromSamplingDistance(goldPosition);
        initializeTrajectory(0, backupDistance, 0).run();
    }

    private double getBackupFromSamplingDistance(TensorFlow.GoldPosition goldPosition) {
        if (goldPosition == TensorFlow.GoldPosition.LEFT) {
            return BACKUP_DISTANCE_LEFT;
        } else if (goldPosition == TensorFlow.GoldPosition.RIGHT) {
            return BACKUP_DISTANCE_RIGHT;
        } else {
            return BACKUP_DISTANCE_CENTER;
        }
    }

    public void driveToWall(TensorFlow.GoldPosition goldPosition) {
        if (goldPosition == TensorFlow.GoldPosition.LEFT) {
            initializeTrajectory(0, WALL_DISTANCE_LEFT, 0).run();
        } else if (goldPosition == TensorFlow.GoldPosition.RIGHT) {
            initializeTrajectory(0, WALL_DISTANCE_RIGHT, 0).run();
        } else {
            initializeTrajectory(0, WALL_DISTANCE_CENTER, 0).run();
        }
    }

    public void alignWithWall(Robot.StartingPosition startingPosition) {
        if(startingPosition == Robot.StartingPosition.DEPOT) {
            alignWithWallDepot();
        } else {
            alignWithWallCrater();
        }
    }

    private void alignWithWallDepot() {
        initializeTrajectory(DISTANCE_TO_WALL, 0,DEPOT_WALL_HEADING).run();
        initializeTrajectory(-DISTANCE_AWAY_FROM_WALL, 0, DEPOT_WALL_HEADING).run();
    }

    private void alignWithWallCrater() {
        initializeTrajectory(-DISTANCE_TO_WALL, 0, CRATER_WALL_HEADING).run();
        initializeTrajectory(DISTANCE_AWAY_FROM_WALL, 0, CRATER_WALL_HEADING).run();
    }

    public void driveToDepot(Robot.StartingPosition startingPosition) {
        if (startingPosition == Robot.StartingPosition.DEPOT) {
            initializeTrajectory(0, DEPOT_TO_DEPOT_DISTANCE, DEPOT_WALL_HEADING).run();
        } else {
            initializeTrajectory(0, CRATER_TO_DEPOT_DISTANCE, CRATER_WALL_HEADING).run();
        }
    }

    public void setPowers(double magnitude, double x, double y, double heading) {
        double[] powers = mecanumEnhanced.calculatePowers(magnitude, x, y, heading);
        setIndividualPowers(powers);
    }

    public void turnToHeading(double headingError) {
        double kP = 0.065;
        double leftPower = PIDController.pController(0, headingError, -kP);
        double rightPower = PIDController.pController(0, headingError, kP);
        setIndividualPowers(leftPower, leftPower, rightPower, rightPower);
    }

    public void turnToHeadingLeftWheels(double headingError) {
        double kP = 0.07;
        double leftPower = PIDController.pController(0, headingError, -kP);
        setIndividualPowers(leftPower, leftPower, 0, 0);
    }

    public void turnToHeadingRightWheels(double headingError) {
        double kP = 0.07;
        double rightPower = PIDController.pController(0, headingError, kP);
        setIndividualPowers(0, 0, rightPower, rightPower);
    }

    public void setIndividualPowers(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower){
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    public void setIndividualPowers(double[] powerArray){
        setIndividualPowers(powerArray[0], powerArray[1], powerArray[2], powerArray[3]);
    }

    public void stop() {
        setIndividualPowers(0, 0, 0, 0);
    }


    public MecanumTrajectoryFollower initializeTrajectory(double x, double y, double heading) {
        TrajectoryGenerator trajectory = new TrajectoryGenerator(x, y, 1, MAX_ACCEL);
        return new MecanumTrajectoryFollower(allMotors, trajectory, heading, kA, false);
    }

    public MecanumTrajectoryFollower initializeTrajectory(double x, double y, double heading, double maxAccel, boolean usesFeedback) {
        TrajectoryGenerator trajectory = new TrajectoryGenerator(x, y, 1, maxAccel);
        return new MecanumTrajectoryFollower(allMotors, trajectory, heading, kA, usesFeedback);
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

    public void runToPosition(int targetPosition, double power) {
        MotorEnhanced.setTargetPosition(allMotors, targetPosition);
        MotorEnhanced.setRunMode(allMotors, DcMotor.RunMode.RUN_TO_POSITION);
        MotorEnhanced.setPower(allMotors, power);
    }

    public void unlatch() {
        unlatchAwayFromLander.run();
        unlatchParallelToLander.run();
        driveAwayFromLander.run();
        centerForSampling.run();
    }

    public void park(Robot.StartingPosition startingPosition) {
        if (startingPosition == Robot.StartingPosition.DEPOT) {
            initializeTrajectory(0, PARKING_DISTANCE, DEPOT_WALL_HEADING).run();
        } else {
            initializeTrajectory(0, PARKING_DISTANCE, CRATER_WALL_HEADING).run();
        }
    }

    public void driveAwayFromMarker() {
        driveAwayFromMarker.run();
    }

    public void driveAwayFromLander() {
        driveAwayFromLander.run();
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

    public double convertEncoderCountsToInches(int encoderCounts) {
        return encoderCounts / ENCODER_COUNTS_PER_ROTATION * DISTANCE_PER_REVOLUTION;
    }

    public double getMaxStrafeVelocity() {
        return MAX_STRAFE_VELOCITY;
    }

    public double getMaxForwardVelocity() {
        return MAX_FORWARD_VELOCITY;
    }

    public DcMotor[] getAllMotors() {
        return allMotors;
    }

    public static Drive getInstance() {
        return instance;
    }

}

