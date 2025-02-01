package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

public class speedyServos {
    Servo wrist;
    OpMode opmode;
    RobotConfig config;
    public static double subIntake = 0;
    public static double deposit = 0.85;
    public static double subEntrance = 0.2;
    public double targetPos = 0;

    public speedyServos(OpMode opmode, RobotConfig config){
        this.opmode = opmode;
        this.config = config;
        wrist = opmode.hardwareMap.get(Servo.class, config.deviceConfig.wristServo);
    }

    public void deposit(){
        wrist.setPosition(deposit);
        targetPos = deposit;
    }

    public void enterSub(){
        wrist.setPosition(subEntrance);
        targetPos = subEntrance;
    }

    public void Intake(){
        wrist.setPosition(subIntake);
        targetPos = subIntake;
    }

    public void inSub(double pos){
        double maxRot = 0.425;
        wrist.setPosition(abs(pos) * maxRot);
        targetPos = (abs(pos) * maxRot);
    }

    public double getWristPos(){
        return targetPos;
    }
}
