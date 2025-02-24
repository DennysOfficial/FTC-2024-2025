package org.firstinspires.ftc.teamcode.RobotStuff;

import android.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftPivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveSpecimenClaw;

public class SpecimenArm {

    public static SpecimenArmPose restPose = new SpecimenArmPose(0.5, 0, -80);
    public static double resetPresetDurationSec = 1;
    public static SpecimenArmPose scorePose = new SpecimenArmPose(0.5, 10, 32);
    public static double scorePresetDurationSec = 1;
    public static SpecimenArmPose collectPose = new SpecimenArmPose(0.5, 2, -80);
    public static double collectPresetDurationSec = 1;

    public static Range<Double> scorePivotDeadZone = new Range<>(10.0, scorePose.pivotPosition);

    enum SpecimenArmState {
        collect,
        rest,
        score,
    }

    SpecimenArmState armState = SpecimenArmState.rest;
    SpecimenArmState previousState = null;


    OpMode opmode;
    RobotConfig config;

    LeftLift leftLift;
    LeftPivot otherSpinnyBit;

    ActiveSpecimenClaw claw;


    public SpecimenArm(OpMode opMode, RobotConfig config) {
        this.opmode = opMode;
        this.config = config;

        leftLift = new LeftLift(ControlAxis.ControlMode.positionControl, opmode, config);
        otherSpinnyBit = new LeftPivot(ControlAxis.ControlMode.positionControl, opmode, config);

        otherSpinnyBit.assignLift(leftLift);
        leftLift.assignPivot(otherSpinnyBit);

        claw = new ActiveSpecimenClaw(opMode, config);
    }


    public void update() {
        if (config.debugConfig.getStateDebug())
            opmode.telemetry.addData("Harpoon Arm State: ", armState.toString());

        updateState();
        if (previousState != armState) {
            switch (armState) {
                case rest:
                    moveToPose(restPose, resetPresetDurationSec);
                    break;

                case score:
                    moveToPose(scorePose, scorePresetDurationSec);
                    break;

                case collect:
                    moveToPose(collectPose, collectPresetDurationSec);
                    break;
            }
        }

        switch (armState) {
            case score:
                // if (scorePivotDeadZone.contains(otherSpinnyBit.getPosition()))

                break;
        }
        leftLift.update();
        otherSpinnyBit.update();
        previousState = armState;
    }

    void updateState() {
        if (config.inputMap.getSpecimenHangButton())
            armState = SpecimenArmState.score;

        if (config.inputMap.getSpecimenCollectButton())
            armState = SpecimenArmState.collect;

        if (config.inputMap.getSpecimenRestButton())
            armState = SpecimenArmState.rest;
    }


    public void moveToPose(SpecimenArmPose pose, double duration) {
        leftLift.setControlMode(ControlAxis.ControlMode.positionControl);
        leftLift.setTargetPosition(pose.liftPosition);
        otherSpinnyBit.fancyMoveToPosition(pose.liftPosition, duration);
        claw.setWristPosition(pose.wristPosition);
    }
}
