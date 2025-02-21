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

    public static double elbowPosScore = 0.3;
    public static double wristPosScore = 0.12;
    public static double liftPosScore = 5.12;
    public static double pivotPosScore = 21.15;

    public static double elbowPosCollect = 0.8;
    public static double wristPosCollect = 0.3;
    public static double liftPosCollect = 5.12; //TODO: TEST AND CHANGE
    public static double pivotPosCollect = 21.15; //TODO: TEST AND CHANGE

    public static double elbowPosRest = 0; //TODO: TEST AND CHANGE
    public static double wristPosRest = 0; //TODO: TEST AND CHANGE
    public static double liftPosRest = 5.12; //TODO: TEST AND CHANGE
    public static double pivotPosRest = 21.15; //TODO: TEST AND CHANGE


    public PassiveGrabber(OpMode opmode, RobotConfig config, Lift lift, Pivot pivot) {
        this.opmode = opmode;
        this.config = config;
        this.lift = lift;
        this.pivot = pivot;
        elbow = opmode.hardwareMap.get(Servo.class, config.deviceConfig.elbowServo);
        wrist = opmode.hardwareMap.get(Servo.class, config.deviceConfig.wristServo);
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
