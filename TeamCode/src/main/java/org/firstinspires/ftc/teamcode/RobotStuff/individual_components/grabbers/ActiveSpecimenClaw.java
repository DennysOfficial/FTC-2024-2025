package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.SpecimenArm;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftPivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.RightPivot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

public class ActiveSpecimenClaw {

    Servo wristServo;
    Servo pinchServo;


    public static double hardClosedPosition = .269f;
    public static double softClosedPosition = .269f;
    public static double openPosition = .6f;


    OpMode opmode;
    RobotConfig config;

    LeftLift lift;
    LeftPivot pivot;

    public ActiveSpecimenClaw(OpMode opMode, RobotConfig config) {
        this.opmode = opMode;
        this.config = config;

        wristServo = opmode.hardwareMap.get(Servo.class, config.deviceConfig.specimenWristServo);
        pinchServo = opmode.hardwareMap.get(Servo.class, config.deviceConfig.specimenClawServo);
    }

    public ActiveSpecimenClaw(OpMode opMode, RobotConfig config, LeftLift lift, LeftPivot pivot) {
        this.opmode = opMode;
        this.config = config;

        this.lift = lift;
        this.pivot = pivot;

        wristServo = opmode.hardwareMap.get(Servo.class, config.deviceConfig.specimenWristServo);
        pinchServo = opmode.hardwareMap.get(Servo.class, config.deviceConfig.specimenClawServo);
    }

    public void basicGampadPinchControl() {
        if (config.inputMap.getSpecimenClawOpen())
            pinchServo.setPosition(openPosition);
        else
            pinchServo.setPosition(hardClosedPosition);
    }

    public void openClaw() {
        pinchServo.setPosition(openPosition);
    }

    public void closeClawHard() {
        pinchServo.setPosition(hardClosedPosition);
    }

    public void closeClawSoft() {
        pinchServo.setPosition(softClosedPosition);
    }

    public void setWristPosition(double position) {
        wristServo.setPosition(position);
    }

    public void Score() {
        pivot.setTargetPosition(SpecimenArm.scorePose.pivotPosition);
        lift.setTargetPosition(SpecimenArm.scorePose.liftPosition);
        wristServo.setPosition(SpecimenArm.scorePose.wristPosition);
    }

    public void Collect() {
        pivot.setTargetPosition(SpecimenArm.collectPose.pivotPosition);
        lift.setTargetPosition(SpecimenArm.collectPose.liftPosition);
        wristServo.setPosition(SpecimenArm.collectPose.wristPosition);
    }

    public void rest() {
        pivot.setTargetPosition(SpecimenArm.restPose.pivotPosition);
        lift.setTargetPosition(SpecimenArm.restPose.liftPosition);
        wristServo.setPosition(SpecimenArm.restPose.wristPosition);
    }

}
