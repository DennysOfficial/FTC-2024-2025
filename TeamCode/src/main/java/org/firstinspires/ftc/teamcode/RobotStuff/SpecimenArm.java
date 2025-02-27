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
    public static SpecimenArmPose collectPose = new SpecimenArmPose(0.62, 1, -55);
    public static double collectPresetDurationSec = 1;

    public static double kPForwards = 0.008;
    public static double kPBackwards = 0.08;
    public static double damping = 0.001;

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
                if (!otherSpinnyBit.isBusy())
                    armState = SpecimenArmState.score;
                break;
            case score:
                // if (scorePivotDeadZone.contains(otherSpinnyBit.getPosition()))
                updatePidCoefficients();
                otherSpinnyBit.setControlMode(ControlAxis.ControlMode.torqueControl);
                otherSpinnyBit.targetTorque = scoringPid.runPID(scorePose.pivotPosition, otherSpinnyBit.getPosition(), otherSpinnyBit.deltaTime);
                break;
        }

        claw.basicGampadPinchControl();

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
