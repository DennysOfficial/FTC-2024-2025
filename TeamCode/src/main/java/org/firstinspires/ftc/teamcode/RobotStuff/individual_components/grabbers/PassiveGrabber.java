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

    public static double elbowPos = 0.12;
    public static double wristPos = 0;

    public PassiveGrabber(OpMode opmode, RobotConfig config) {
        this.opmode = opmode;
        this.config = config;
        elbow = opmode.hardwareMap.get(Servo.class, config.deviceConfig.elbowServo);
        wrist = opmode.hardwareMap.get(Servo.class, config.deviceConfig.wristServo);
    }

    public void Score() {
        wristPos = 0.12;
        elbowPos = 0.3;
    }

    public void Collect() {
        wristPos = 0.8;
        elbowPos = 0.3; // test & change with ftc dashboard
    }

    public void moveElbow() { //can't move the elbow outside of the file so here
        if (config.inputMap.getElbowRight() == config.inputMap.getElbowLeft()) {
            elbowPos += 0;
        }
        else if (config.inputMap.getElbowLeft()) {
            elbowPos += 0.01;
        }
        else if (config.inputMap.getElbowRight()) {
            elbowPos -= 0.01;
        }
    }


    public double getElbowPos() {
        return elbowPos;
    }


    public void Update() {
        elbow.setPosition(elbowPos);
        wrist.setPosition(wristPos);
    }
}
