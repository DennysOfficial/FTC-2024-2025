package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

@Config
public class PassiveGrabber {

    Servo elbow;
    Servo wrist;

    OpMode opmode;
    RobotConfig config;

    public static double elbowPosRest = 0.15;
    public static double wristPosRest = 0.83;

    public static double elbowPosRest1 = 0.5;

    public static double wristPosScore = 0.83;
    public static double elbowPosScore = 0.46;

    public static double wristPosCollect = 0.16;
    public static double elbowPosCollect = 0.57;

    public PassiveGrabber(OpMode opmode, RobotConfig config) {
        this.opmode = opmode;
        this.config = config;
        elbow = opmode.hardwareMap.get(Servo.class, config.deviceConfig.elbowServo);
        wrist = opmode.hardwareMap.get(Servo.class, config.deviceConfig.spWristServo);
    }

    public void Score() {
        wrist.setPosition(wristPosScore);
        elbow.setPosition(elbowPosScore);
    }

    public void Collect() {
        wrist.setPosition(wristPosCollect);
        elbow.setPosition(elbowPosCollect);
    }

    public void Rest() {
        wrist.setPosition(wristPosRest);
        elbow.setPosition(elbowPosRest);
    }

    public void setPosition(double elbowPos, double wristPos) {
        wrist.setPosition(wristPos);
        elbow.setPosition(elbowPos);
    }

    public void setElbowPos(double elbowPos) {
        elbow.setPosition(elbowPos);
    }

    public void setWristPos(double wristPos) {
        wrist.setPosition(wristPos);
    }

    public double getElbowPos() {
        return elbow.getPosition();
    }

    public double getWristPos() {
        return wrist.getPosition();
    }

    public double getElbowPosRest1() {
        return elbowPosRest1;
    }
}
