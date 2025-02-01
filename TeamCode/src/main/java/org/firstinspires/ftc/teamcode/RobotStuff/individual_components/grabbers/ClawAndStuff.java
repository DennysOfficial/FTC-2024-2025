package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
@Config
public class ClawAndStuff {

    OpMode opMode;
    RobotConfig config;


    Servo pinchServo;
    public static double closedPosition = .6f;
    public static double openPosition = .4f;

    Servo twistServo;
    public static double twistUp = .6;
    public static double twistDown = .4;


    public ClawAndStuff(OpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;
        pinchServo = opMode.hardwareMap.get(Servo.class, config.deviceConfig.clawServo);
        twistServo = opMode.hardwareMap.get(Servo.class, config.deviceConfig.clawTwistServo);
    }

    void updatePincher() {
        if (config.inputMap.getClawOpen())
            pinchServo.setPosition(openPosition);
        else
            pinchServo.setPosition(closedPosition);
    }
}
