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
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ArmIK;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.MathStuff;

@Config
public class HarpoonArm {

    public static double grabPosition = 0;

    public static double ObservationDepositArmAngle = -60;
    public static double ObservationWristPosition = 0.5;
    public static double ObservationDepositLiftPosition = 0;

    public static double HighBasketDepositArmAngle = -10;
    public static double HighBasketWristPosition = 0.769;
    public static double HighBasketDepositLiftPosition = 20;


    public static double StoreWristPosition = 0.3;
    public static double StoreArmAngle = -70;
    public static double StoreLiftPosition = 0;

    /**
     * relative to the center of the pivot
     */
    public static double intakeLiftExtension = 0;
    public static double intakeTorque = 0.4;
    public static double IntakeWristPosition = 0.3;
    public static double clawTriggerHeightOffsetExtended = 2.7;
    public static double clawTriggerHeightOffsetRetracted = 2.7;
    double interpolationExtendedLiftDistance = 20;

    double clawTriggerHeightOffset() {
        return MathStuff.lerp(clawTriggerHeightOffsetRetracted, clawTriggerHeightOffsetExtended, rightLift.getPosition() / interpolationExtendedLiftDistance);
    }

    public static double clawTriggerScale = -1;

    /**
     * the distance between the axis of rotation and the line in the direction of extension through the controlled point
     */
    public static double extensionAxisOffset = 4.2;
    public static double intakeHeightOffset = -1.5;

    public static double intakeAngleOffset = -3.5;


    final OpMode opMode;
    final RobotConfig config;

    final RightLift rightLift;
    final RightPivot rightPivot;

    final Harpoon harpoon;

    ArmIK armIK = new ArmIK();

    public enum SampleArmState {
        positionControl,
        store,
        intakeHeightBasedGrab,
        intakeTimeBasedGrab,
        depositLow,
        depositHigh,
    }

    SampleArmState armState = SampleArmState.store;
    SampleArmState previousArmState = null;

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

        previousArmState = armState;

        //opMode.telemetry.addData("target intake pivot angle = %f", calculateIntakePivotAngle());
        //opMode.telemetry.addData("intake height = %f", calculateIntakeHeight());

        if (config.debugConfig.getStateDebug())
            opMode.telemetry.addData("Harpoon Arm State: ", armState.toString());
    }

    boolean grabOpen = true;

    boolean lastGroundSlam = false;

    void updatePresets() {
        if (config.inputMap.getIntakeForward())
            armState = SampleArmState.intakeHeightBasedGrab;

        if (config.inputMap.getObservationDepositPreset())
            armState = SampleArmState.depositLow;

        if (config.inputMap.getBasketDepositPreset())
            armState = SampleArmState.depositHigh;


        if (config.inputMap.getClawCloseButton())
            grabOpen = false;
        if (config.inputMap.getClawOpenButton())
            grabOpen = true;


        if (armState != previousArmState) {
            switch (armState) {
                case store:
                    harpoon.setWristPosition(StoreWristPosition);
                    rightPivot.fancyMoveToPosition(StoreArmAngle, 1);

                    rightLift.setControlMode(ControlAxis.ControlMode.gamePadVelocityControl);
                    rightLift.setTargetPosition(StoreLiftPosition);


                    rightLift.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;
                    rightPivot.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;
                    armState = SampleArmState.positionControl;
                    break;

                case depositLow:
                    harpoon.setWristPosition(ObservationWristPosition);
                    rightPivot.fancyMoveToPosition(ObservationDepositArmAngle, 1);

                    rightLift.setControlMode(ControlAxis.ControlMode.gamePadVelocityControl);
                    rightLift.setTargetPosition(ObservationDepositLiftPosition);


                    rightLift.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;
                    rightPivot.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;
                    armState = SampleArmState.positionControl;
                    break;

                case depositHigh:
                    harpoon.setWristPosition(HighBasketWristPosition);
                    rightPivot.fancyMoveToPosition(HighBasketDepositArmAngle, 1);

                    rightLift.setControlMode(ControlAxis.ControlMode.gamePadVelocityControl);
                    rightLift.setTargetPosition(HighBasketDepositLiftPosition);


                    rightLift.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;
                    rightPivot.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;
                    armState = SampleArmState.positionControl;
                    break;

                case intakeHeightBasedGrab:
                    harpoon.setWristPosition(IntakeWristPosition);
                    rightPivot.fancyMoveToPosition(calculateIntakePivotAngle(intakeLiftExtension), 1);

                    rightLift.setControlMode(ControlAxis.ControlMode.gamePadVelocityControl);
                    rightLift.setTargetPosition(intakeLiftExtension);

                    rightLift.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;
                    rightPivot.defaultControlMode = ControlAxis.ControlMode.positionControl;
                    break;
            }
        }
        switch (armState) {
            case intakeHeightBasedGrab:
                if (config.inputMap.getYoinkButton()
                        && rightPivot.getControlMode() != ControlAxis.ControlMode.disabled
                        && rightPivot.getPosition() > 55) {

                    rightPivot.targetTorque = intakeTorque;
                    rightPivot.setControlMode(ControlAxis.ControlMode.torqueControl);


                    if (config.inputMap.getClawCloseButton()) // emergency override
                        harpoon.setGrabPosition(1);
                    else if (config.inputMap.getClawOpenButton())
                        harpoon.setGrabPosition(0);
                    else
                        harpoon.setGrabPosition((calculateIntakeHeight() + clawTriggerHeightOffset()) * clawTriggerScale);

                    grabOpen = false;
                    lastGroundSlam = true;

                    break;
                }

                if (lastGroundSlam) { // if the last update was a ground slam go back to intake height less violently
                    lastGroundSlam = false;
                    rightPivot.fancyMoveToPosition(calculateIntakePivotAngle(), 0.420);
                }

                if (!rightPivot.isBusy()) {
                    rightPivot.setControlMode(ControlAxis.ControlMode.positionControl);
                    rightPivot.setTargetPosition(calculateIntakePivotAngle());
                }

            default:
                if (grabOpen)
                    harpoon.setGrabPosition(0);
                else
                    harpoon.setGrabPosition(1);
        }
    }

    public double calculateIntakePivotAngle() {
        return intakeAngleOffset + armIK.getTargetAngle(rightLift.retractedExtension + rightLift.getPosition(), extensionAxisOffset, intakeHeightOffset);
    }

    public double calculateIntakePivotAngle(double liftExtension) {
        return intakeAngleOffset + armIK.getTargetAngle(rightLift.retractedExtension + liftExtension, extensionAxisOffset, intakeHeightOffset);
    }

    /**
     * relative to the center of the pivot
     */
    public double calculateIntakeHeight() {
        return armIK.getHeight(rightPivot.getPosition() - intakeAngleOffset, rightLift.retractedExtension + rightLift.getPosition(), extensionAxisOffset);
    }

}
