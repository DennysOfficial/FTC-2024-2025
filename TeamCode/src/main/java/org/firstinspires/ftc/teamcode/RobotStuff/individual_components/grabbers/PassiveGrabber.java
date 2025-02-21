package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftPivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.RightLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.RightPivot;

@Config
public class PassiveGrabber {

    Servo elbow;
    Servo wrist;

    LeftLift lift;
    LeftPivot pivot;

    OpMode opmode;
    RobotConfig config;

    public static double elbowPosRest = 0.15;
    public static double wristPosRest = 0.83;
    public static double liftPosRest = 0;
    public static double pivotPosRest = -80;

    public static double wristPosScore = 0.83;
    public static double elbowPosScore = 0.469;
    public static double liftPosScore = 6.8;
    public static double pivotPosScore = 30;

    public static double wristPosCollect = 0.16;
    public static double elbowPosCollect = 0.415;
    public static double liftPosCollect = 0;
    public static double pivotPosCollect = -67;

    public PassiveGrabber(OpMode opmode, RobotConfig config, LeftLift lift, LeftPivot pivot) {
        this.opmode = opmode;
        this.config = config;
        this.lift = lift;
        this.pivot = pivot;
        elbow = opmode.hardwareMap.get(Servo.class, config.deviceConfig.elbowServo);
        wrist = opmode.hardwareMap.get(Servo.class, config.deviceConfig.spWristServo);
    }

    public void Score() {
        wrist.setPosition(wristPosScore);
        elbow.setPosition(elbowPosScore);
        lift.setTargetPosition(liftPosScore);
        pivot.setTargetPosition(pivotPosScore);
    }

    public void Collect() {
        wrist.setPosition(wristPosCollect);
        elbow.setPosition(elbowPosCollect);
        lift.setTargetPosition(liftPosCollect);
        pivot.setTargetPosition(pivotPosCollect);
    }

    public void Rest() {
        wrist.setPosition(wristPosRest);
        elbow.setPosition(elbowPosRest);
        lift.setTargetPosition(liftPosRest);
        pivot.setTargetPosition(pivotPosRest);
    }
}
