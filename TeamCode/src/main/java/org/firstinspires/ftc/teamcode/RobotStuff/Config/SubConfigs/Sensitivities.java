package org.firstinspires.ftc.teamcode.RobotStuff.Config.SubConfigs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Sensitivities {

    public static float driveSensitivity = 1;

    public float getDriveSensitivity() {
        return driveSensitivity;
    }

    public static float forwardSensitivity = 1; // basic driving sensitivities only relative to each other

    public float getForwardSensitivity() {
        return forwardSensitivity;
    }


    public static float turningSensitivity = .69f;

    public float getTurningSensitivity() {
        return turningSensitivity;
    }


    public static float turningRateDPS = 130;

    public float getTurningRateDPS() {
        return turningRateDPS;
    }

    public static float slowTurningRateDPS = 30;

    public float getSlowTurningRateDPS() {
        return slowTurningRateDPS;
    }


    public static float strafingSensitivity = 1;

    public float getStrafingSensitivity() {
        return strafingSensitivity;
    }


    public static float liftRate = 10; // inches per second

    public float getLiftRate() {
        return liftRate;
    }

    public static float liftSensitivity = 1;

    public float getLiftSensitivity() {
        return liftSensitivity;
    }

    public static float pivotRate = 50; //degrees per second

    public float getPivotRate() {
        return pivotRate;
    }

    public static float pivotSensitivity = 0.45f; //thingamabobs per yugvjkhjblk

    public float getPivotSensitivity() {
        return pivotSensitivity;
    }

    public static float slowDownModifier = 0.4f;

    public float getSlowDownModifier(){return slowDownModifier;}

    public static float maxGoDownAmount = 0.3f;

    public float getMaxGoDownAmount(){
        return maxGoDownAmount;
    }

}
