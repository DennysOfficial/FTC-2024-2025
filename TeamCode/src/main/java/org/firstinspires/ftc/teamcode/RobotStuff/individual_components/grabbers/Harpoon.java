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
    RobotConfig config;

    public static double openPos = 0.25, closePos = 0.48;


    // public static double frontPos = 0.5, SidePos = 0.5, backPos = 0.5;


    OpMode opMode;

    public Harpoon(OpMode opMode, RobotConfig config) {
        this.config = config;
        this.opMode = opMode;

        harpoonServo = opMode.hardwareMap.get(Servo.class, config.deviceConfig.harpoonGrabServo);
        wristServo = opMode.hardwareMap.get(Servo.class, config.deviceConfig.harpoonDepositWristServo);
    }

    /**
     * ranges from 0 - 1   for open - close
     */
    public void setGrabPosition(double position) {
        position = MathUtils.clamp(position,0,1);
        position = MathStuff.map(position,0,1,openPos,closePos);
        harpoonServo.setPosition(position);
    }


}
