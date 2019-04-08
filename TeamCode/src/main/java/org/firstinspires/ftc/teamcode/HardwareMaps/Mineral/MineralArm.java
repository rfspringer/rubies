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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Library.AccelerationController;
import org.firstinspires.ftc.teamcode.Library.PivotTrajectoryFollower;
import org.firstinspires.ftc.teamcode.Library.TrajectoryFollower;
import org.firstinspires.ftc.teamcode.Library.TrajectoryGenerator;

/**
 * This class stores all objects on our robot's drivetrain
 * It also includes functionality specific to our drive base
 */
public class MineralArm {
    private static final MineralArm instance = new MineralArm();

    private AccelerationController pivotAccelerationControl = new AccelerationController(0.75);
    /* Public OpMode members. */
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;

    private double ENCODER_COUNTS_PER_RADIAN = ;
    private double MAX_VELOCITY = ;
    private double MAX_ACCELERATION = ;

    private double kV = 0.5/MAX_VELOCITY;
    private double kA = 0;
    private double kAExternal = 0;

    /* local OpMode members. */
    private HardwareMap hwMap = null;

    /* Constructor */
    private MineralArm(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        motor1 = hwMap.dcMotor.get("arm1");
        motor2 = hwMap.dcMotor.get("arm2");
        initializeMotor(motor1);
        initializeMotor(motor2);
    }

    private void initializeMotor(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setPower(0);
    }

    public PivotTrajectoryFollower initializeTrajectory(double targetPositionInDegrees) {
        double trajectoryLength = Math.toRadians(targetPositionInDegrees - getAngle());
        TrajectoryGenerator trajectory = new TrajectoryGenerator(trajectoryLength, MAX_VELOCITY, MAX_ACCELERATION);
        return new PivotTrajectoryFollower(getMotors(), trajectory, kV, kA, kAExternal);
    }

    /**
     * @return position of arm in radians
     */
    public double getAngle() {
        return motor1.getCurrentPosition() * ENCODER_COUNTS_PER_RADIAN;
    }

    public double getVelocity() {
        return (getAngle() - previousAngle)/(timer.seconds - previousTime);
    }

    public void setPowers(double power) {
        DcMotor[] motors = {motor1, motor2};
        pivotAccelerationControl.run(power, motors);
    }

    public double getPower() {
        return motor1.getPower();
    }

    public DcMotor[] getMotors() {
        return new DcMotor[] {motor1, motor2};
    }

    public static MineralArm getInstance(){
        return instance;
    }

}

