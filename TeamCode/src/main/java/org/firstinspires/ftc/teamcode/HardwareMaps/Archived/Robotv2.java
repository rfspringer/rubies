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

package org.firstinspires.ftc.teamcode.HardwareMaps.Archived;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareMaps.Claim;
import org.firstinspires.ftc.teamcode.HardwareMaps.Lift;
import org.firstinspires.ftc.teamcode.HardwareMaps.Sensors;
import org.firstinspires.ftc.teamcode.Library.PIDController;
import org.firstinspires.ftc.teamcode.Library.TensorFlow;

import static java.lang.Thread.sleep;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for our robot
 * This class stores functions that 3use a combination of subsystems on our robot
 */
public class Robotv2 {
    private static final Robotv2 instance = new Robotv2();
    public Drivev2 drive = Drivev2.getInstance();
    public Lift lift = Lift.getInstance();
    public Mineralv2 mineral = Mineralv2.getInstance();
    public Claim claim = Claim.getInstance();
    public Sensors sensors = Sensors.getInstance();

    /* Constructor */
    private Robotv2(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        drive.init(hwMap);
        lift.init(hwMap);
        mineral.init(hwMap);
        sensors.init(hwMap);
        claim.init(hwMap);
    }

    public void driveByHeading(double leftPower, double rightPower, double targetHeading) {
        double kP = 0.0065;
        double error = targetHeading - sensors.getHeading();
        double left = PIDController.pController(leftPower, error, -kP);
        double right = PIDController.pController(rightPower, error, kP);
        drive.setPowers(leftPower, rightPower);
    }

    public void turnToHeadingCenterPivot(double targetHeading) {
        while (Math.abs(sensors.integrateHeading(targetHeading - sensors.getHeading())) > 2.5) {
            double kP = 0.0065;
            double error = targetHeading - sensors.getHeading();
            double leftPower = PIDController.pController(0, error, -kP);
            double rightPower = PIDController.pController(0, error, kP);
            drive.setPowers(leftPower, rightPower);
        }
        drive.setPowers(0, 0);
    }


    public void turnToHeadingForwardPivot(double targetHeading) {
        while (Math.abs(sensors.integrateHeading(targetHeading - sensors.getHeading())) > 2.5) {
            double kP = 0.0065;
            double error = targetHeading - sensors.getHeading();
            double leftPower = PIDController.pController(0, error, -kP);
            double rightPower = PIDController.pController(0, error, kP);
            if (leftPower > 0) {
                drive.setPowers(leftPower, 0);
            } else {
                drive.setPowers(0, rightPower);
            }
        }
        drive.setPowers(0, 0);
    }

    public void turnToHeadingBackwardPivot(double targetHeading) {
        while (Math.abs(sensors.integrateHeading(targetHeading - sensors.getHeading())) > 2.5) {
            double kP = 0.0065;
            double error = targetHeading - sensors.getHeading();
            double leftPower = PIDController.pController(0, error, -kP);
            double rightPower = PIDController.pController(0, error, kP);
            if (leftPower < 0) {
                drive.setPowers(leftPower, 0);
            } else {
                drive.setPowers(0, rightPower);
            }
        }
        drive.setPowers(0, 0);
    }

    public void sample(TensorFlow.GoldPosition goldLocation) {
        if (goldLocation == TensorFlow.GoldPosition.LEFT) {
            turnToHeadingCenterPivot(sensors.getLeftMineralHeading());
            drive.driveToLeftMineral.run();
        } else if (goldLocation == TensorFlow.GoldPosition.RIGHT) {
            turnToHeadingCenterPivot(sensors.getRightMineralHeading());
            drive.driveToRightMineral.run();
        } else {
            turnToHeadingCenterPivot(sensors.getCenterMineralHeading());
            drive.driveToCenterMineral.run();
        }
    }


    public void claim(TensorFlow.GoldPosition goldLocation) {
        if (goldLocation == TensorFlow.GoldPosition.LEFT) {
            turnToHeadingBackwardPivot(-35);
            drive.driveFromLeftMineral.run();
        } else if (goldLocation == TensorFlow.GoldPosition.RIGHT) {
            turnToHeadingBackwardPivot(35);
            drive.driveFromRightMineral.run();
        } else {
            turnToHeadingBackwardPivot(15);
            drive.driveFromCenterMineral.run();
        }
        claim.deploy();
    }

    public static Robotv2 getInstance() {
        return instance;
    }
 }

