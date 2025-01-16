package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

public class PassiveGrabber {

    Servo elbow;
    Servo wrist;

    OpMode opmode;
    RobotConfig config;

    double elbowPos = 0.12;
    double wristPos = 0;

    public PassiveGrabber(OpMode opmode, RobotConfig config) {
        this.opmode = opmode;
        this.config = config;
        elbow = opmode.hardwareMap.get(Servo.class, config.deviceConfig.elbowServo);
        wrist = opmode.hardwareMap.get(Servo.class, config.deviceConfig.spWristServo);
    }

    public void Score() {
        wristPos = 0.8;
        elbowPos = 0.3;
    }

    public void Collect() {
        wristPos = 0.12;
    }

    public void setPosition(double elbowPos, double wristPos) {
        this.elbowPos = elbowPos;
        this.wristPos = wristPos;
    }

    public void setElbowPos(double elbowPos) {
        if (elbowPos >= 1) {
            elbowPos = 1;
        } else if (elbowPos <= 0) {
            elbowPos = 0;
        }
        this.elbowPos = elbowPos;
    }

    public void setWristPos(double wristPos) {
        if (wristPos >= 1) {
            wristPos = 1;
        } else if (wristPos <= 0) {
            wristPos = 0;
        }
        this.wristPos = wristPos;
    }

    public double getElbowPos() {
        return elbowPos;
    }

    public double getWristPos() {
        return wristPos;
    }

    public void Update() {
        elbow.setPosition(elbowPos);
        wrist.setPosition(wristPos);
    }
}
