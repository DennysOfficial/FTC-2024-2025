/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.RobotStuff;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.RightLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.RightPivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.Harpoon;

@Config
public class HarpoonArm {

    public static double grabPosition = 0;

    public static double ObservationDepositArmAngle = -70;
    public static double ObservationDepositLiftPosition = 0;

    public static double StoreArmAngle = -70;
    public static double StoreLiftPosition = 0;

    /**
     * relative to the center of the pivot
     */
    public static double intakeLiftExtension = 0;
    public static double intakeTorque = 0.2;
    public static double clawTriggerHeightOffset = -2;
    public static double clawTriggerScale = 1;

    /**
     * the distance between the axis of rotation and the line in the direction of extension through the controlled point
     */
    public static double extensionAxisOffset = 5;
    public static double intakeHeightOffset = -1;

    public static double intakeAngleOffset = -4;


    final OpMode opMode;
    final RobotConfig config;

    final RightLift rightLift;
    final RightPivot rightPivot;

    final Harpoon harpoon;

    ArmIK armIK = new ArmIK();

    public enum ArmState {
        store,
        intakeHeightBasedGrab,
        intakeTimeBasedGrab,
        depositLow,
        depositHigh,
    }

    ArmState armState = ArmState.store;
    ArmState previousArmState;

    public HarpoonArm(OpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;

        rightLift = new RightLift(ControlAxis.ControlMode.gamePadVelocityControl, opMode, config);
        rightPivot = new RightPivot(ControlAxis.ControlMode.gamePadVelocityControl, opMode, config);

        rightLift.assignPivot(rightPivot);
        rightPivot.assignLift(rightLift);

        harpoon = new Harpoon(opMode, config);
    }

    public void update() {
        updatePresets();
        rightLift.update();
        rightPivot.update();

        opMode.telemetry.addData("target intake pivot angle = %f", calculateIntakePivotAngle());
        opMode.telemetry.addData("intake height = %f", calculateIntakeHeight());
        previousArmState = armState;
    }

    void updatePresets() {
        if (config.inputMap.getIntakeForward())
            armState = ArmState.intakeHeightBasedGrab;

        if (config.inputMap.getObservationDepositPreset())
            armState = ArmState.depositLow;

        if (armState != previousArmState) {
            switch (armState) {
                case store:
                    rightPivot.fancyMoveToPosition(StoreArmAngle, 1);
                    rightLift.fancyMoveToPosition(StoreLiftPosition, 0.75);
                    rightLift.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;
                    rightPivot.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;
                    break;
                case depositLow:
                    rightPivot.fancyMoveToPosition(ObservationDepositArmAngle, 1);
                    rightLift.fancyMoveToPosition(ObservationDepositLiftPosition, 0.75);
                    rightLift.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;
                    rightPivot.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;
                    break;
                case intakeHeightBasedGrab:
                    rightPivot.fancyMoveToPosition(calculateIntakePivotAngle(), 1);
                    rightLift.fancyMoveToPosition(intakeLiftExtension, 0.75);

                    rightLift.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;
                    rightPivot.defaultControlMode = ControlAxis.ControlMode.positionControl;
                    break;
            }
        } else
            switch (armState) {
                case intakeHeightBasedGrab:
                    rightPivot.setTargetPosition(calculateIntakePivotAngle());

                    if (config.inputMap.getYoinkButton()
                            && rightPivot.getControlMode() != ControlAxis.ControlMode.disabled
                            && !rightPivot.isBusy()
                            && rightPivot.getPosition() > 55) {

                        rightPivot.targetTorque = intakeTorque;
                        rightPivot.setControlMode(ControlAxis.ControlMode.torqueControl);

                        harpoon.setGrabPosition((calculateIntakeHeight() + clawTriggerHeightOffset) * clawTriggerScale);
                    } else if (!rightPivot.isBusy()) {
                        rightPivot.setControlModeUnsafe(rightPivot.defaultControlMode);
                        harpoon.setGrabPosition(grabPosition);
                    }
                    break;
            }
    }

    public double calculateIntakePivotAngle() {
        return intakeAngleOffset + armIK.getTargetAngle(rightLift.retractedExtension + rightLift.getPosition(), extensionAxisOffset, intakeHeightOffset);
    }

    /**
     * relative to the center of the pivot
     */
    public double calculateIntakeHeight() {
        return -(rightLift.retractedExtension + rightLift.getPosition()) * Math.sin(Math.toRadians(90 - rightPivot.getPosition()));
    }

}
