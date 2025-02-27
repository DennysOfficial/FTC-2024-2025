package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.AngleServo;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.MathStuff;
@Config
public class SampleClaw {

    Servo harpoonServo;
    Servo bigWristServo;
    AngleServo smolWristServo;
    RobotConfig config;

    public static double openPos = .6, closePos = 0.8420, rTwist = 1, lTwist = 0.69, mTwist = 0.69;


    // public static double frontPos = 0.5, SidePos = 0.5, backPos = 0.5;


    OpMode opMode;

    public SampleClaw(OpMode opMode, RobotConfig config) {
        this.config = config;
        this.opMode = opMode;

        harpoonServo = opMode.hardwareMap.get(Servo.class, config.deviceConfig.harpoonGrabServo);
        bigWristServo = opMode.hardwareMap.get(Servo.class, config.deviceConfig.harpoonDepositWristServo);
        smolWristServo = new AngleServo(config.deviceConfig.harpoonBlockAlignmentWristServo,opMode.hardwareMap,0,-100,1,100);
    }

    /**
     * ranges from 0 - 1   for open - close
     */
    public void  setGrabPosition(double position) {
        position = MathUtils.clamp(position,0,1);
        position = MathStuff.map(position,0,1,openPos,closePos);
        harpoonServo.setPosition(position);
    }

    public void setBigWristPosition(double position){
        bigWristServo.setPosition(position);
    }

    public void setSmolWristPosition(double position){
        smolWristServo.setPosition(position);
    }

    public void twistServo(double pos){
        if (pos == 1){
            smolWristServo.setPosition(rTwist);
        }
        if (pos == 0){
            smolWristServo.setPosition(mTwist);
        }
        if (pos == -1){
            smolWristServo.setPosition(lTwist);
        }
    }

}
