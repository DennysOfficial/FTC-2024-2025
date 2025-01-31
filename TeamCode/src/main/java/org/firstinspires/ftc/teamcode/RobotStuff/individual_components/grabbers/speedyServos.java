package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers;

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


    public speedyServos(OpMode opmode, RobotConfig config){
        this.opmode = opmode;
        this.config = config;
        wrist = opmode.hardwareMap.get(Servo.class, config.deviceConfig.wristServo);
    }

    public void deposit(){
        wrist.setPosition(deposit);
    }

    public void enterSub(){
        wrist.setPosition(subEntrance);
    }

    public void Intake(){
        wrist.setPosition(subIntake);
    }

    public void inSub(double pos){
        wrist.setPosition(pos);
    }

}
