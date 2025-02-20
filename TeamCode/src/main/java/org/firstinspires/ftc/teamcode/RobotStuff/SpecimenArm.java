package org.firstinspires.ftc.teamcode.RobotStuff;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftPivot;

public class SpecimenArm {

    public static double wristPosRest = 0.5;
    public static double liftPosRest = 0;
    public static double pivotPosRest = -80;

    public static double wristPosScore = 0.5;
    public static double liftPosScore = 11.3;
    public static double pivotPosScore = 32;

    public static double wristPosCollect = 0.5;
    public static double liftPosCollect = 2;
    public static double pivotPosCollect = -80;

    enum SpecimenArmState {
        collect,
        rest,
        score,
    }

    SpecimenArmState armState = SpecimenArmState.rest;


    OpMode opmode;
    RobotConfig config;

    LeftLift leftLift;

    LeftPivot otherSpinnyBit;


    SpecimenArm(OpMode opMode, RobotConfig config) {
        this.opmode = opMode;
        this.config = config;

        leftLift = new LeftLift(ControlAxis.ControlMode.positionControl, opmode, config);
        otherSpinnyBit = new LeftPivot(ControlAxis.ControlMode.positionControl, opmode, config);

        otherSpinnyBit.assignLift(leftLift);
        leftLift.assignPivot(otherSpinnyBit);
    }

    public void update() {


        leftLift.update();
        otherSpinnyBit.update();
    }

    void updatePresets() {
        if (config.inputMap.getSpecimenHangButton())
            armState = SpecimenArmState.score;

        if (config.inputMap.getSpecimenCollectButton())
            armState = SpecimenArmState.collect;

        if (config.inputMap.getSpecimenRestButton())
            armState = SpecimenArmState.rest;

    }


}
