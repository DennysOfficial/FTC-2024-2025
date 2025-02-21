package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Lift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Pivot;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

@Config
public class PassiveGrabber {


    Servo elbow;
    Servo wrist;

    OpMode opmode;
    RobotConfig config;

    Lift lift;
    Pivot pivot;
    // yay it work

    public static boolean reverseScore = false;

    public static double elbowPosScore = 0.585;
    public static double wristPosScore = 0.155;
    public static double liftPosScore = 3.8;
    public static double pivotPosScore = 21.15;

    public static double elbowPosReverseScore = 0;
    public static double wristPosReverseScore = 0.8;
    public static double liftPosReverseScore = 13.6;
    public static double pivotPosReverseScore = 15;

    public static double elbowPosCollect = 0.36;
    public static double wristPosCollect = 0.155;
    public static double liftPosCollect = 0;
    public static double pivotPosCollect = 36;

    public static double elbowPosRest = 0.16;
    public static double wristPosRest = 0.155;
    public static double liftPosRest = 0;
    public static double pivotPosRest = -39;


    public PassiveGrabber(OpMode opmode, RobotConfig config, Lift lift, Pivot pivot) {
        this.opmode = opmode;
        this.config = config;
        this.lift = lift;
        this.pivot = pivot;
        elbow = opmode.hardwareMap.get(Servo.class, config.deviceConfig.elbowServo);
        wrist = opmode.hardwareMap.get(Servo.class, config.deviceConfig.wristServo);
    }

    public void Score() {
        if (reverseScore) {
            wrist.setPosition(wristPosReverseScore);
            elbow.setPosition(elbowPosReverseScore);
            lift.fancyMoveToPosition(liftPosReverseScore, 0.5);
            pivot.fancyMoveToPosition(pivotPosReverseScore, 0.5);
        } else {
            wrist.setPosition(wristPosScore);
            elbow.setPosition(elbowPosScore);
            lift.fancyMoveToPosition(liftPosScore, 0.5);
            pivot.fancyMoveToPosition(pivotPosScore, 0.5);
        }
    }

    public void Collect() {
        wrist.setPosition(wristPosCollect);
        elbow.setPosition(elbowPosCollect);
        lift.fancyMoveToPosition(liftPosCollect, 0.5);
        pivot.fancyMoveToPosition(pivotPosCollect, 0.5);
    }

    public void Rest() {
        wrist.setPosition(wristPosRest);
        elbow.setPosition(elbowPosRest);
        lift.fancyMoveToPosition(liftPosRest,1);
        pivot.fancyMoveToPosition(pivotPosRest, 1);
    }
}
