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

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Library.Archived.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.Other.MineralWaypoint;

/**
 * This class stores all objects on our robot's drivetrain
 * It also includes functionality specific to our drive base
 */
public class Mineral {
    private static final Mineral instance = new Mineral();
    private MineralPivot pivot = MineralPivot.getInstance();
    private MineralIntake intake = MineralIntake.getInstance();
    private MineralExtension extension = MineralExtension.getInstance();

    private HardwareMap hwMap;

    private int MASS;change //in kilograms
    private int NUMBER_OF_TRAJECTORY_WAYPOINTS;change

    /* Constructor */
    private Mineral(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        pivot.init(hwMap);
        intake.init(hwMap);
        extension.init(hwMap);
    }

    public void setPowers(double armPower, double extensionPower) {
        setArmPower(armPower);
        setExtensionPower(extensionPower);
    }

    public MineralWaypoint[] createTrajectory(double targetAngle, double targetLength) {
        MineralWaypoint[] trajectory1 = createTrajectory(extension.getCurrentLength(), pivot.getCurrentAngle(),
                        targetAngle, 1);
        MineralWaypoint[] trajectory2 = createTrajectory(targetLength, targetAngle,
                pivot.getCurrentAngle(), -1);
        return getActualTrajectory(trajectory1, trajectory2);
    }

    private MineralWaypoint[] getActualTrajectory(MineralWaypoint[] trajectory1, MineralWaypoint[] trajectory2) {
        for (int i = 0; i < trajectory1.length; i++) {
            if (trajectory2[i].getLength() > trajectory1[i].getLength()) {
                trajectory1[i] = trajectory2[i];
            }
        }
        return trajectory1;
    }

    private MineralWaypoint[] createTrajectory(double initialLength, double initialAngle, double targetAngle, double targetExtensionPower) {
        MineralWaypoint[] trajectory = initializeTrajectory(initialLength, initialAngle);
        double dTheta = getDTheta(initialAngle, targetAngle);
        TrajectoryGenerator pivotTrajectory = pivot.createTrajectory(initialAngle, targetAngle);
        return addWaypointsToTrajectory(trajectory, dTheta, pivotTrajectory, initialLength, targetExtensionPower);
    }

    private MineralWaypoint[] addWaypointsToTrajectory(MineralWaypoint[] trajectory, double dTheta, TrajectoryGenerator pivotTrajectory, double initialLength, double targetExtensionPower) {
        double previousExtensionPower = 0;
        double time = 0;
        double previousLength = initialLength;
        for (int i = 1; i < trajectory.length; i++) {
            MineralWaypoint waypoint = createWaypoint(i, time, dTheta, pivotTrajectory, previousExtensionPower, previousLength, targetExtensionPower);
            trajectory[i] = waypoint;
            previousExtensionPower = waypoint.getExtensionPower();
            time += waypoint.getdTime();
            previousLength = waypoint.getLength();
        }
        return trajectory;
    }

    private MineralWaypoint[] initializeTrajectory(double initialLength, double initialAngle) {
        MineralWaypoint[] trajectory = new MineralWaypoint[NUMBER_OF_TRAJECTORY_WAYPOINTS];
        MineralWaypoint initialWaypoint = new MineralWaypoint(0, initialLength, initialAngle,
                0, 0, 0);
        trajectory[0] = initialWaypoint;
        return trajectory;
    }

    private MineralWaypoint createWaypoint(int waypointSegment, double time, double dTheta, TrajectoryGenerator pivotTrajectory, double previousExtensionPower, double previousLength, double targetExtensionPower) {
        double targetAngularVelocity = pivotTrajectory.getVelocityIfMaxAccel(time);
        double dTime = dTheta/targetAngularVelocity;
        double extensionPower = pivot.getAccelerationControlledPower(dTime, previousExtensionPower, targetExtensionPower); // update accel controller to allow
        double length = extension.calculateLengthFromIntegration(previousLength, extensionPower, dTime);   //will integrate to find (previous length += dTime * current velocity)
        double angle = dTheta * waypointSegment;
        double inertia = extension.getMomentofInertia(length, MASS);
        double torque = getTorqueFromGravity(angle);
        double pivotPower = pivot.getPower(targetAngularVelocity, torque, inertia);
        return new MineralWaypoint(angle, dTime, length, extensionPower, pivotPower, targetAngularVelocity);
    }

    private double getTorqueFromGravity(double angle) {
        return extension.getPositionOfCenterOfGravity() * pivot.getForceFromGravity(angle, MASS);
    }

    private double getDTheta(double initialAngle, double targetAngle) {
        return (targetAngle - initialAngle)/NUMBER_OF_TRAJECTORY_WAYPOINTS;
    }

    public void setToIntake() {
        intake.setToIntake();
    }

    public void setExtensionPower(double power) {
        extension.setPower(power);
    }

    public void dumpMinerals() {
        intake.dumpMinerals();
    }

    public void storeMinerals() {
        intake.storeMinerals();
    }

    public void setScaledPower(double power) {
        intake.setScaledPower(power);
    }

    public double getScaledPower() {
        return intake.getScaledPower();
    }

    public void setArmPower(double power){
        pivot.setPowers(power);
    }

    public void setIntakeRawPower(double power) {
        intake.setRawPower(power);
    }

    public void setIntakeScaledPower(double power) {
        intake.setScaledPower(power);
    }

    public static Mineral getInstance() {
        return instance;
    }
}

