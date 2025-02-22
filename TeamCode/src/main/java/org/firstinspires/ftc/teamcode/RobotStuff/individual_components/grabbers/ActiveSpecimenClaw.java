package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

public class ActiveSpecimenClaw {

    Servo wristServo;
    Servo pinchServo;


    public static double hardClosedPosition = .269f;
    public static double softClosedPosition = .269f;
    public static double openPosition = .6f;


    OpMode opmode;
    RobotConfig config;

    public ActiveSpecimenClaw(OpMode opMode, RobotConfig config) {
        this.opmode = opMode;
        this.config = config;

        wristServo = opmode.hardwareMap.get(Servo.class, config.deviceConfig.spWristServo);
        pinchServo = opmode.hardwareMap.get(Servo.class, config.deviceConfig.clawServo);
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


}
