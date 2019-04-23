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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Library.RubiesLinearOpMode;
import org.firstinspires.ftc.teamcode.Library.VexMotorEnhanced;

/**
 * This class stores all objects on our robot's lift
 * It also includes functionality specific to our lift
 */
public class Lift {
    private static final Lift instance = new Lift();
    private RubiesLinearOpMode opMode;
    /* Public OpMode members. */
    private DcMotor  lift   = null;
    private CRServo pin = null;

    /* local OpMode members. */
    private HardwareMap hwMap = null;

    private int EXTENDED_ENCODER_COUNTS = -5100;

    /* Constructor */
    private Lift(RubiesLinearOpMode opMode){
        this.opMode = opMode;
    }

    /* Constructor */
    private Lift(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, RubiesLinearOpMode opMode) {
        this.opMode = opMode;
        init(ahwMap);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        initializeMotor();
        initializeServo();
    }

    private void initializeMotor() {
        lift = hwMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setPower(0);
    }

    private void initializeServo() {
        pin = hwMap.crservo.get("pin");
        pin.setPower(0);
    }

    public void lower() {
        removePinAutonomously();
        stopPin();
        extendLiftAutonomously();
        stopLift();
    }

    private void removePinAutonomously() {
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < 1.5) {
            addTelemetryToAvoidDisconnect();
            setPower(1);
            removePin();
        }
    }

    private void extendLiftAutonomously() {
        ElapsedTime timer = new ElapsedTime();
        while (!robotIsCloseToGround(timer)) {
            addTelemetryToAvoidDisconnect();
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setPower(-0.1);
        }
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopLift();
    }

    private boolean robotIsCloseToGround(ElapsedTime time) {
        return (getCurrentPosition() <= (EXTENDED_ENCODER_COUNTS )) || time.seconds() > 3.5;
    }

    private void stopLift() {
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setPower(0);
    }

    public void removePin() {
        VexMotorEnhanced.setScaledPower(pin, -1);
    }

    public void stopPin() {
        VexMotorEnhanced.setScaledPower(pin, 0);
    }

    public void setPower(double power) {
        lift.setPower(power);
    }

    public DcMotor getMotor() {
        return lift;
    }

    public int getCurrentPosition() {
        return lift.getCurrentPosition();
    }

    public void setTargetPosition(int targetPosition) {
        lift.setTargetPosition(targetPosition);
    }

    public void setMode(DcMotor.RunMode runMode) {
        lift.setMode(runMode);
    }

    public void holdHangingPosition() {
        setPower(0.2);
    }

    private void addTelemetryToAvoidDisconnect() {
        opMode.telemetry.addData("status", "running");
        opMode.telemetry.update();
    }

    public static Lift getInstance(){
        return instance;
    }
}

