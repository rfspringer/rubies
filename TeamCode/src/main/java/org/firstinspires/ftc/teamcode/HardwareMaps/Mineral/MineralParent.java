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

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class stores all objects on our robot's drivetrain
 * It also includes functionality specific to our drive base
 */
public class MineralParent {
    private static final MineralParent instance = new MineralParent();
    private MineralPivot pivot = MineralPivot.getInstance();
    private MineralIntake intake = MineralIntake.getInstance();
    private MineralExtension extension = MineralExtension.getInstance();

    private HardwareMap hwMap;

    /* Constructor */
    private MineralParent(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        pivot.init(hwMap);
        intake.init(hwMap);
        extension.init(hwMap);
    }

    //COLLECTOR

    public void setToIntake() {
        intake.setToIntake();
    }

    public double getIntakeScaledPower() {
        return intake.getScaledPower();
    }

    public void setIntakeRawPower(double power) {
        intake.setRawPower(power);
    }

    public void setIntakeScaledPower(double power) {
        intake.setScaledPower(power);
    }

    public void dumpMinerals() {
        intake.releaseMinerals();
    }

    public void storeMinerals() {
        intake.storeMinerals();
    }

    //PIVOT

    public void setPivotPower(double power){
        pivot.setPowers(power);
    }

    public double getPivotPower() {
        return pivot.getPower();
    }

    public boolean limitSwitchIsPressed() {
        return pivot.isPressed();
    }

    public double getLimitSwitchReading() {
        return pivot.getReading();
    }
    //EXTENSION

    public void setExtensionPower(double power) {
        extension.setPower(power);
    }

    public static MineralParent getInstance() {
        return instance;
    }
}

