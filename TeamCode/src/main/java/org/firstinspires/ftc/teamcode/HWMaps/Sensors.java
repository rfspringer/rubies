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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for our robot
 * This class stores functions that use a combination of subsystems on our robot
 */
public class Sensors
{
    private static final Sensors instance = new Sensors();

    private BNO055IMU imu;
    private Orientation angles;
    private Acceleration gravity;
    private boolean hasSetInitialAngle = false;
    private double initialHeading;

    private double IMU_WALL_OFFSET = 45.0;

    private double CENTER_MINERAL_HEADING = 10;
    private double LEFT_MINERAL_HEADING = 40;
    private double RIGHT_MINERAL_HEADING = -16.5;

    /* Constructor */
    private Sensors(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        imu = hwMap.get(BNO055IMU.class, "imu");

        initializeIMU();
        updateIMU();
    }

    private void initializeIMU() {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    public void updateIMU() {
        //Updates everything
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();

        //initialize "initialHeading" value the first time through the loop (again, sometimes our imu doesn't zero when we reset it every time, we do this to prevent the issue
        if (!hasSetInitialAngle){
            initialHeading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) - IMU_WALL_OFFSET;
            hasSetInitialAngle = true;
        }
    }

    /*
    Gets heading and integrates to be between -180 and 180 just in case
     */
    public double getHeading(){
        //Gets heading from imu
        double rawHeading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) + initialHeading;

        return integrateHeading(rawHeading);
    }

    public double integrateHeading(double heading){
        //Integrates it to be from -180 to 180 degrees
        while (heading > 180){
            heading = heading - 360;
        }
        while (heading < -180){
            heading = heading + 360;
        }

        return heading;
    }

    public double getCenterMineralHeading() {
        return CENTER_MINERAL_HEADING;
    }

    public double getLeftMineralHeading() {
        return LEFT_MINERAL_HEADING;
    }

    public double getRightMineralHeading() {
        return RIGHT_MINERAL_HEADING;
    }

    public static Sensors getInstance() {
        return instance;
    }
}
