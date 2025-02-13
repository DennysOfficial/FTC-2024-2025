package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

public class speedyServos {
    Servo wrist;
    OpMode opmode;
    RobotConfig config;
    public static double subIntake = 0.2;
    public static double deposit = 0.85;
    public static double subEntrance = 0.425;
    public double targetPos = 0;
    public boolean hold = false;
    public boolean inSub = false;
    public speedyServos(OpMode opmode, RobotConfig config){
        this.opmode = opmode;
        this.config = config;
        wrist = opmode.hardwareMap.get(Servo.class, config.deviceConfig.wristServo);
    }

    public void deposit(){
        wrist.setPosition(deposit);
        targetPos = deposit;
        inSub = false;
    }

    public void enterSub(){
        wrist.setPosition(subEntrance);
        targetPos = subEntrance;
        inSub = false;
    }

    public void Intake(){
        wrist.setPosition(subIntake);
        targetPos = subIntake;
    }

    public boolean inSubRout(double pos){
        if (!hold && pos > 0.2){
            hold = true;
        }
        if (pos < 0.2 && hold){
            hold = false;
            inSub = true;
            return true;
        }
        return false;
    }

    public double getWristPos(){
        return targetPos;
    }
}
