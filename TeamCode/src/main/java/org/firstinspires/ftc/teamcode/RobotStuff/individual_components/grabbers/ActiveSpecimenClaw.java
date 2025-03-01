package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

@Config
public class ActiveSpecimenClaw {

    Servo wristServo;
    Servo pinchServo;
    public static double hardClosedPosition = 0.269;
    public static double softClosedPosition = 0.3;
    public static double openPosition = 0.69;


    OpMode opmode;
    RobotConfig config;

    public ActiveSpecimenClaw(OpMode opMode, RobotConfig config) {
        this.opmode = opMode;
        this.config = config;

        wristServo = opmode.hardwareMap.get(Servo.class, config.deviceConfig.specimenWristServo);
        pinchServo = opmode.hardwareMap.get(Servo.class, config.deviceConfig.specimenClawServo);
    }

    public void basicGampadPinchControlHardClose() {
        if (config.inputMap.getSpecimenClawOpen())
            pinchServo.setPosition(openPosition);
        else
            pinchServo.setPosition(hardClosedPosition);
    }
    public void basicGampadPinchControlSoftClose() {
        if (config.inputMap.getSpecimenClawOpen())
            pinchServo.setPosition(openPosition);
        else
            pinchServo.setPosition(softClosedPosition);
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

    public void clawPos(double position) {pinchServo.setPosition(position);}
}
