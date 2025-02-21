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
    public static double elbowPosScore = 0.6;
    public static double wristPosScore = 0.17;
    public static double liftPosScore = 3.5;
    public static double pivotPosScore = 21.15;

    public static double elbowPosCollect = 0.37;
    public static double wristPosCollect = 0.17;
    public static double liftPosCollect = 0;
    public static double pivotPosCollect = 47;

    public static double elbowPosRest = 0.16;
    public static double wristPosRest = 0.17;
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
