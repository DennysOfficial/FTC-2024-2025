package org.firstinspires.ftc.teamcode.RobotStuff.individual_components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

public class DoubleArm {

    private armGroup leftArm;
    private armGroup rightArm;

    OpMode opmode;
    RobotConfig config;

    private armGroup activeArm;
    private armGroup requestedActiveArm;
    private String activeArmName;
    private boolean queuedArm;

    public armGroup getActiveArm() {
        return activeArm;
    }

    public DoubleArm(OpMode opmode, RobotConfig config) {
        this.config = config;
        this.opmode = opmode;
    }


    public void setLeftArm(armGroup arm) {
        leftArm = arm;
        activeArm = leftArm;
        activeArmName = arm.name;
    }

    public void setRightArm(armGroup arm) {
        rightArm = arm;
        activeArm = rightArm;
        activeArmName = arm.name;
    }


    public void setActiveArm(armGroup arm) {
        activeArm.deactivate(); // deactiate current arm
        activeArm = arm; // set the active arm (to the other arm)
        activeArmName = activeArm.name;
        activeArm.activate(); // activate the other arm
    }


    public void updateArm() {
        if (config.playerTwo.setRightArm.getRisingState()) {
            requestedActiveArm = rightArm;
            queuedArm = true;
        }

        if (config.playerTwo.setLeftArm.getRisingState()) {
            requestedActiveArm = leftArm;
            queuedArm = true;
        }

        if (queuedArm) {setActiveArm(requestedActiveArm);}
        queuedArm = false;

        opmode.telemetry.addData("Active Arm", activeArmName);

        leftArm.update();
        rightArm.update();
    }
}
