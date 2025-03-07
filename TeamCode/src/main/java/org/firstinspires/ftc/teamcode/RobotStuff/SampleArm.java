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
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.SampleClaw;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ArmIK;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.MathStuff;

import java.util.Objects;

@Config
public class SampleArm {

    public static double grabPosition = 0;

    public static double a_ClawSmolOpenPosition = 0.5;

    public static SampleArmPose ObservationDepositPose = new SampleArmPose(90, .1, 0, -40);
    public static double DefaultObservationDepositDuration = 1;

    public static SampleArmPose HighBasketDepositPose = new SampleArmPose(0, 1, 32, -20);
    public static double DefaultHighBasketDepositPresetDuration = 1;

    public static SampleArmPose RestPose = new SampleArmPose(0, .42, 0, -40);
    public static double DefaultRestPresetDuration = 1;

    public static double DefaultIntakePresetDuration = 1;
    public static double FromHighBasketIntakePresetDuration = 1.69;

    /**
     * relative to the center of the pivot
     */
    public static double maxIntakeTorque = 0.4;


    public static SampleArmPose IntakeInitialPose = new SampleArmPose(0, .420, 0, Double.NaN);
    public static double intakeHeightOffset = -2;

    public static double clawTriggerHeightOffsetExtended = 4;
    public static double clawTriggerHeightOffsetRetracted = 4;
    public static double clawTriggerScale = -4.20;


    double interpolationExtendedLiftDistance = 20;

    double clawTriggerHeightOffset() {
        return MathStuff.lerp(clawTriggerHeightOffsetRetracted, clawTriggerHeightOffsetExtended, rightLift.getPosition() / interpolationExtendedLiftDistance);
    }

    /**
     * the distance between the axis of rotation and the line in the direction of extension through the controlled point
     */
    public static double extensionAxisOffset = 4.2;

    public static double intakeAngleOffset = -3.5;


    final OpMode opMode;
    final RobotConfig config;

    final RightLift rightLift;
    final RightPivot rightPivot;

    final SampleClaw sampleClaw;

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

    public SampleArm(OpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;

        rightLift = new RightLift(ControlAxis.ControlMode.gamePadVelocityControl, opMode, config);
        rightPivot = new RightPivot(ControlAxis.ControlMode.gamePadVelocityControl, opMode, config);

        rightLift.assignPivot(rightPivot);
        rightPivot.assignLift(rightLift);

        sampleClaw = new SampleClaw(opMode, config);
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

    enum GrabPosition {
        closed,
        bigOpen,
        smolOpen
    }

    GrabPosition clawTargetPosition = GrabPosition.closed;
    boolean lastGroundSlam = false;

    void updateTargetState() {
        if (config.inputMap.getIntakeForward()) {
            armState = SampleArmState.intakeHeightBasedGrab;
            grabPosition = 0;
        }

        if (config.inputMap.getObservationDepositPreset())
            armState = SampleArmState.depositLow;

        if (config.inputMap.getBasketDepositPreset())
            armState = SampleArmState.depositHigh;
    }

    void updatePresets() {

        updateTargetState();

        sampleClaw.blockAlignmentUpdate();

        if (config.inputMap.getClawBigOpenButton())
            clawTargetPosition = GrabPosition.bigOpen;
        if (config.inputMap.getClawSmallOpenButton())
            clawTargetPosition = GrabPosition.smolOpen;
        if (config.inputMap.getClawCloseButton())
            clawTargetPosition = GrabPosition.closed;


        if (armState != previousArmState)
            switch (armState) {
                case store:
                    moveToPose(RestPose, DefaultRestPresetDuration);

                    rightLift.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;
                    rightPivot.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;

                    armState = SampleArmState.positionControl;
                    break;

                case depositLow:
                    moveToPose(ObservationDepositPose, DefaultObservationDepositDuration);

                    rightLift.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;
                    rightPivot.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;

                    armState = SampleArmState.positionControl;
                    break;

                case depositHigh:
                    moveToPose(HighBasketDepositPose, DefaultHighBasketDepositPresetDuration);

                    rightLift.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;
                    rightPivot.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;

                    armState = SampleArmState.positionControl;
                    break;

                case intakeHeightBasedGrab:
                    IntakeInitialPose.pivotPosition = calculateIntakePivotAngle(IntakeInitialPose.liftPosition);

                    if (Objects.requireNonNull(previousArmState) == SampleArmState.depositHigh) {
                        moveToPose(IntakeInitialPose, FromHighBasketIntakePresetDuration);
                    } else {
                        moveToPose(IntakeInitialPose, DefaultIntakePresetDuration);
                    }

                    rightLift.defaultControlMode = ControlAxis.ControlMode.gamePadVelocityControl;
                    rightPivot.defaultControlMode = ControlAxis.ControlMode.positionControl;
                    break;
            }

        switch (armState) {
            case intakeHeightBasedGrab:
                if (config.inputMap.getYoinkButton()
                        && rightPivot.getControlMode() != ControlAxis.ControlMode.disabled
                        && rightPivot.getPosition() > 55) {

                    rightPivot.targetTorque = maxIntakeTorque * config.inputMap.getYoinkTrigger();
                    rightPivot.setControlMode(ControlAxis.ControlMode.torqueControl);


                    if (config.inputMap.getClawCloseButton()) // emergency override
                        sampleClaw.setGrabPosition(1);
                    else if (config.inputMap.getClawBigOpenButton())
                        sampleClaw.setGrabPosition(0);
                    else
                        sampleClaw.setGrabPosition((calculateIntakeHeight() + clawTriggerHeightOffset()) * clawTriggerScale);

                    clawTargetPosition = GrabPosition.closed;
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
                switch (clawTargetPosition){
                    case closed:
                        sampleClaw.setGrabPosition(0);
                    case bigOpen:
                        sampleClaw.setGrabPosition(1);
                    case smolOpen:
                        sampleClaw.setGrabPosition(a_ClawSmolOpenPosition);
                }

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

    public void moveToPose(SampleArmPose pose, double duration) {
        rightLift.linearMoveToPosition(pose.liftPosition, duration);
        rightPivot.fancyMoveToPosition(pose.pivotPosition, duration);
        sampleClaw.setBigWristPosition(pose.wristBigTwistAlignmentPosition);
        sampleClaw.setSmolWristPosition(pose.wristLittleTwistAlignmentAngle);
    }

    public double getPivotPos() {
        return rightPivot.getPosition();
    }

    public double getLiftPos() {
        return rightLift.getPosition();
    }

    public boolean isBusy() {
        return rightLift.isBusy() || rightPivot.isBusy();
    }
}
