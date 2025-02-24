package org.firstinspires.ftc.teamcode.RobotStuff;

import android.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftPivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveSpecimenClaw;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.GoofyPID;

public class SpecimenArm {

    public static SpecimenArmPose restPose = new SpecimenArmPose(0.5, 0, -80);
    public static double resetPresetDurationSec = 1;
    public static SpecimenArmPose scorePose = new SpecimenArmPose(0.5, 10, 32);
    public static double scorePresetDurationSec = 1;
    public static SpecimenArmPose collectPose = new SpecimenArmPose(0.5, 2, -80);
    public static double collectPresetDurationSec = 1;

    public static double kPForwards = 0;
    public static double kPBackwards = 0;
    public static double damping = 0;

    public static Range<Double> scorePivotDeadZone = new Range<>(10.0, scorePose.pivotPosition);

    enum SpecimenArmState {
        collect,
        rest,
        movingToScore,
        score,
    }

    SpecimenArmState armState = SpecimenArmState.rest;
    SpecimenArmState previousState = null;


    OpMode opmode;
    RobotConfig config;

    LeftLift leftLift;
    LeftPivot otherSpinnyBit;

    ActiveSpecimenClaw claw;

    GoofyPID scoringPid;


    public SpecimenArm(OpMode opMode, RobotConfig config) {
        this.opmode = opMode;
        this.config = config;

        leftLift = new LeftLift(ControlAxis.ControlMode.positionControl, opmode, config);
        otherSpinnyBit = new LeftPivot(ControlAxis.ControlMode.positionControl, opmode, config);

        otherSpinnyBit.assignLift(leftLift);
        leftLift.assignPivot(otherSpinnyBit);

        claw = new ActiveSpecimenClaw(opMode, config);

        scoringPid = new GoofyPID(opMode.telemetry, config, "specimen scoring PID");
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

                case movingToScore:
                    moveToPose(scorePose, scorePresetDurationSec);
                    break;

                case score:
                    otherSpinnyBit.setControlMode(ControlAxis.ControlMode.torqueControl);
                    break;

                case collect:
                    moveToPose(collectPose, collectPresetDurationSec);
                    break;
            }
        }

        switch (armState) {
            case movingToScore:
                if (!leftLift.isBusy())
                    armState = SpecimenArmState.score;
            case score:
                // if (scorePivotDeadZone.contains(otherSpinnyBit.getPosition()))
                otherSpinnyBit.targetTorque = scoringPid.runPID(scorePose.pivotPosition, otherSpinnyBit.getPosition(), otherSpinnyBit.deltaTime);
                break;
        }

        leftLift.update();
        otherSpinnyBit.update();
        previousState = armState;
    }

    void updateState() {
        if (config.inputMap.getSpecimenHangButton())
            armState = SpecimenArmState.movingToScore;

        if (config.inputMap.getSpecimenCollectButton())
            armState = SpecimenArmState.collect;

        if (config.inputMap.getSpecimenRestButton())
            armState = SpecimenArmState.rest;
    }


    public void moveToPose(SpecimenArmPose pose, double duration) {
        leftLift.setControlMode(ControlAxis.ControlMode.positionControl);
        leftLift.setTargetPosition(pose.liftPosition);
        otherSpinnyBit.fancyMoveToPosition(pose.pivotPosition, duration);
        claw.setWristPosition(pose.wristPosition);
    }
}
