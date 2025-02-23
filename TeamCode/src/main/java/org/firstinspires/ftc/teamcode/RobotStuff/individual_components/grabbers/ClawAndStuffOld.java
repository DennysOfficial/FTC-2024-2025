package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftPivot;

@Config
public class ClawAndStuffOld {

    Servo twistServo;

    LeftLift lift;
    LeftPivot pivot;

    Servo pinchServo;
    public static double closedPosition = .269f;
    public static double openPosition = .6f;

    OpMode opmode;
    RobotConfig config;

    public static double wristPosRest = 0.83;
    public static double liftPosRest = 0;
    public static double pivotPosRest = -80;

    public static double wristPosScore = 0.94;
    public static double liftPosScore = 11.3;
    public static double pivotPosScore = 32;

    public static double wristPosCollect = 0.31;
    public static double liftPosCollect = 2;
    public static double pivotPosCollect = -80;

    public ClawAndStuffOld(OpMode opmode, RobotConfig config, LeftLift lift, LeftPivot pivot) {
        this.opmode = opmode;
        this.config = config;
        this.lift = lift;
        this.pivot = pivot;
        twistServo = opmode.hardwareMap.get(Servo.class, config.deviceConfig.specimenWristServo);
        pinchServo = opmode.hardwareMap.get(Servo.class, config.deviceConfig.specimenClawServo);
    }

    public void Score() {
        twistServo.setPosition(wristPosScore);
        lift.setTargetPosition(liftPosScore);
        pivot.setTargetPosition(pivotPosScore);
    }

    public void Collect() {
        twistServo.setPosition(wristPosCollect);
        lift.setTargetPosition(liftPosCollect);
        pivot.setTargetPosition(pivotPosCollect);
    }

    public void Rest() {
        twistServo.setPosition(wristPosRest);
        lift.setTargetPosition(liftPosRest);
        pivot.setTargetPosition(pivotPosRest);
    }

    public void setTwistServo(double wristPos) {
        twistServo.setPosition(wristPos);
    }

    public void setWristPos(double wristPos) {
        twistServo.setPosition(wristPos);
    }

    public double getWristPos() {
        return twistServo.getPosition();
    }


    public void updatePincher() {
        if (config.inputMap.getSpecimenClawOpen())
            pinchServo.setPosition(openPosition);
        else
            pinchServo.setPosition(closedPosition);
    }
}
