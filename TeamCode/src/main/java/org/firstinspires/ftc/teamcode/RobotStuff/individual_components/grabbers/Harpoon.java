package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.MathStuff;
@Config
public class Harpoon {

    Servo harpoonServo;
    Servo wristServo;
    Servo hba;
    RobotConfig config;

    public static double openPos = .78, closePos = .25;


    // public static double frontPos = 0.5, SidePos = 0.5, backPos = 0.5;


    OpMode opMode;

    public Harpoon(OpMode opMode, RobotConfig config) {
        this.config = config;
        this.opMode = opMode;

        harpoonServo = opMode.hardwareMap.get(Servo.class, config.deviceConfig.harpoonGrabServo);
        wristServo = opMode.hardwareMap.get(Servo.class, config.deviceConfig.harpoonDepositWristServo);
        hba = opMode.hardwareMap.get(Servo.class, config.deviceConfig.harpoonBlockAlignmentWristServo);
    }

    /**
     * ranges from 0 - 1   for open - close
     */
    public void  setGrabPosition(double position) {
        position = MathUtils.clamp(position,0,1);
        position = MathStuff.map(position,0,1,openPos,closePos);
        harpoonServo.setPosition(position);
    }

    public void setWristPosition(double position){
        wristServo.setPosition(position);
    }

    public void twistServo(double pos){
        if (pos == 1){
            hba.setPosition(0.75);
        }
        if (pos == 0){
            hba.setPosition(0.5);
        }
        if (pos == -1){
            hba.setPosition(0.25);
        }
    }

}
