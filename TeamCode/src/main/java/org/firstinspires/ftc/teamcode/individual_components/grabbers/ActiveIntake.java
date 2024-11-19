package org.firstinspires.ftc.teamcode.individual_components.grabbers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;

public class ActiveIntake {

    OpMode opMode;
    RobotConfig config;


    CRServo spinyServo;

    Servo flapServo;

    float intakeSpeed = -1;
    float outtakeSpeed = 1;

    double flapOpen = 0;
    double flapClosed = 0.9;


    public ActiveIntake(OpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;
        spinyServo = opMode.hardwareMap.get(CRServo.class, config.deviceConfig.grabberServo);
        flapServo = opMode.hardwareMap.get(Servo.class, config.deviceConfig.flapServo);
    }


    public void directControl() {
        wheelControl();
        flapControl();
    }

    public void wheelControl(){
        spinyServo.setPower(0);

        if (config.inputMap.getIntakeButton())
            spinyServo.setPower(intakeSpeed);

        else if (config.inputMap.getOuttakeButton())
            spinyServo.setPower(outtakeSpeed);
    }

    public void flapControl(){
        flapServo.setPosition(flapClosed);
        if (config.inputMap.getFlapButton())
            flapServo.setPosition(flapOpen);
    }


}
