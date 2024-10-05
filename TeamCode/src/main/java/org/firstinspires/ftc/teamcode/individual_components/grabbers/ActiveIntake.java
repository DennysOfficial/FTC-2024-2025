package org.firstinspires.ftc.teamcode.individual_components.grabbers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;

public class ActiveIntake {

    LinearOpMode opMode;
    RobotConfig config;


    Servo spinnyServo;

    float intakeSpeed = 0;
    float outtakeSpeed = 1;


    public ActiveIntake(LinearOpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;
        spinnyServo = opMode.hardwareMap.get(Servo.class, config.deviceConfig.grabberServo);
    }


    public void directControl() {
        spinnyServo.setPosition(0.5f);

        if(config.getIntakeButton())
            spinnyServo.setPosition(intakeSpeed);

        else if(config.getOuttakeButton())
            spinnyServo.setPosition(outtakeSpeed);
    }

}
