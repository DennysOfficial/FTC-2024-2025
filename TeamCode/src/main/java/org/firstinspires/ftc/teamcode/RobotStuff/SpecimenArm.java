package org.firstinspires.ftc.teamcode.RobotStuff;

import android.util.Range;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftPivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveSpecimenClaw;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.GoofyPID;
@Config
public class SpecimenArm {


    public static SpecimenArmPose restPose = new SpecimenArmPose(.2, 0, -60);
    public static double resetPresetDurationSec = 1;
    public static SpecimenArmPose scorePose = new SpecimenArmPose(.2, 19, 32);
    public static double scorePresetDurationSec = 1;
    public static SpecimenArmPose collectPose = new SpecimenArmPose(0.65, 2, -50);
    public static double collectPresetDurationSec = 1;

    public static double kPForwards = 0.008;
    public static double kPBackwards = 0.08;
    public static double damping = 0.001;

    public static Range<Double> scorePivotDeadZone = new Range<>(10.0, scorePose.pivotPosition);

    public enum SpecimenArmState {
        collect,
        rest,
        movingToScore,
        score,
    }

    public SpecimenArmState armState = SpecimenArmState.rest;
    SpecimenArmState previousState = null;


    OpMode opmode;
    RobotConfig config;

    LeftLift leftLift;
    LeftPivot otherSpinnyBit;

    ActiveSpecimenClaw claw;
    public void openClaw(){
        claw.openClaw();
    }
    public void closeClawHard(){
        claw.closeClawHard();
    }

    GoofyPID scoringPid;

    void updatePidCoefficients() {
        scoringPid.setCoefficients(kPForwards, kPBackwards, 0, damping);
    }

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


    public void autoUpdate() {
        if (config.debugConfig.getStateDebug())
            opmode.telemetry.addData("Specimen Arm State", armState.toString());

        updateState();

        if (previousState != armState) {
            switch (armState) {
                case rest:
                    moveToPose(restPose, resetPresetDurationSec);
                    break;

                case movingToScore:
                    claw.closeClawSoft();
                    moveToPose(scorePose, scorePresetDurationSec);
                    break;

                case score:
                    if(previousState != SpecimenArmState.movingToScore){
                        armState = SpecimenArmState.movingToScore;
                        return;
                    }

                    otherSpinnyBit.setControlMode(ControlAxis.ControlMode.torqueControl);
                    break;

                case collect:
                    moveToPose(collectPose, collectPresetDurationSec);
                    break;
            }
        }

        switch (armState) {
            case movingToScore:
                if (!otherSpinnyBit.isBusy())
                    armState = SpecimenArmState.score;
                break;
            case score:
                // if (scorePivotDeadZone.contains(otherSpinnyBit.getPosition()))
                updatePidCoefficients();
                otherSpinnyBit.setControlMode(ControlAxis.ControlMode.torqueControl);
                otherSpinnyBit.targetTorque = scoringPid.runPID(scorePose.pivotPosition, otherSpinnyBit.getPosition(), otherSpinnyBit.deltaTime);
                break;
            default:
        }


        leftLift.update();
        otherSpinnyBit.update();
        previousState = armState;
    }

    public void update() {
        if (config.debugConfig.getStateDebug())
            opmode.telemetry.addData("Specimen Arm State", armState.toString());

        updateState();

        if (previousState != armState) {
            switch (armState) {
                case rest:
                    moveToPose(restPose, resetPresetDurationSec);
                    break;

                case movingToScore:
                    claw.closeClawSoft();
                    moveToPose(scorePose, scorePresetDurationSec);
                    break;

                case score:
                    if(previousState != SpecimenArmState.movingToScore){
                        armState = SpecimenArmState.movingToScore;
                        return;
                    }

                    otherSpinnyBit.setControlMode(ControlAxis.ControlMode.torqueControl);
                    break;

                case collect:
                    moveToPose(collectPose, collectPresetDurationSec);
                    break;
            }
        }

        switch (armState) {
            case movingToScore:
                if (!otherSpinnyBit.isBusy())
                    armState = SpecimenArmState.score;
                claw.basicGampadPinchControlSoftClose();
                break;
            case score:
                // if (scorePivotDeadZone.contains(otherSpinnyBit.getPosition()))
                updatePidCoefficients();
                otherSpinnyBit.setControlMode(ControlAxis.ControlMode.torqueControl);
                otherSpinnyBit.targetTorque = scoringPid.runPID(scorePose.pivotPosition, otherSpinnyBit.getPosition(), otherSpinnyBit.deltaTime);
                claw.basicGampadPinchControlSoftClose();
                break;
            default:
                claw.basicGampadPinchControlHardClose();
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


    void moveToPose(SpecimenArmPose pose, double duration) {
        leftLift.setControlMode(ControlAxis.ControlMode.positionControl);
        leftLift.setTargetPosition(pose.liftPosition);
        otherSpinnyBit.fancyMoveToPosition(pose.pivotPosition, duration);
        claw.setWristPosition(pose.wristPosition);
    }

    public boolean pivotIsBusy() {
        return otherSpinnyBit.isBusy();
    }

    public double liftPos() {
        return leftLift.getPosition();
    }
}
