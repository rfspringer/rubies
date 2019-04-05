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

package org.firstinspires.ftc.teamcode.HardwareMaps.Mineral;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Library.VexMotorEnhanced;

/**
 * This class stores all objects on our robot's drivetrain
 * It also includes functionality specific to our drive base
 */
public class MineralIntake {
    private static final MineralIntake instance = new MineralIntake();

    private CRServo intake1;
    private CRServo intake2;
    private Servo   door;
    private HardwareMap hwMap;
    private CRServo[] intake = new CRServo[2];

    private double INTAKE_POWER = 1;
    private double OUTTAKE_POWER = -1;

    private double INTAKE_POSITION = 0.54;
    private double STORAGE_POSITION = 0.52;
    private double RELEASE_POSITION = 0.39;

    /* Constructor */
    private MineralIntake(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        setObjectsFromHardwareMap(hwMap);
        setIntakeDirections();
        door.setPosition(STORAGE_POSITION);
        initializeIntakeArray();
        setRawPower(0);
    }

    private void setIntakeDirections() {
        intake1.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void initializeIntakeArray() {
        intake[0] = intake1;
        intake[1] = intake2;
    }

    private void setObjectsFromHardwareMap(HardwareMap hwMap) {
        intake1 = hwMap.crservo.get("intake1");
        intake2 = hwMap.crservo.get("intake2");
        door = hwMap.servo.get("door");
    }

    public void setToIntake() {
        setScaledPower(1);
        door.setPosition(STORAGE_POSITION);
    }

    public void storeMinerals() {
        door.setPosition(STORAGE_POSITION);
    }

    public void releaseMinerals() {
        door.setPosition(RELEASE_POSITION);
    }

    public void setRawPower(double power) {
        intake1.setPower(power);
        intake2.setPower(power);
    }

    public double getRawPower() {
        return intake1.getPower();
    }

    public void setScaledPower(double power) {
        VexMotorEnhanced.setScaledPower(intake, power);
    }

    public double getScaledPower() {
        return VexMotorEnhanced.getScaledPower(intake1);
    }

    public static MineralIntake getInstance(){
        return instance;
    }
}

