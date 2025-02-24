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

    public static float playerOneLeftXAxisThreshold = 0.0f;

    public static float playerOneLeftYAxisThreshold = 0.0f;

    public static float playerOneRightXAxisThreshold = 0.0f;

    public static float playerOneRightYAxisThreshold = 0.0f;

    public static float playerTwoLeftXAxisThreshold = 0.0f;

    public static float playerTwoLeftYAxisThreshold = 0.0f;

    public static float playerTwoRightXAxisThreshold = 0.0f;

    public static float playerTwoRightYAxisThreshold = 0.0f;

    public static boolean isPlayerOneLeftXInverted = false;

    public static boolean isPlayerOneLeftYInverted = true;

    public static boolean isPlayerOneRightXInverted = false;

    public static boolean isPlayerOneRightYInverted = false;

    public static boolean isPlayerTwoLeftXInverted = false;

    public static boolean isPlayerTwoLeftYInverted = true;

    public static boolean isPlayerTwoRightXInverted = false;

    public static boolean isPlayerTwoRightYInverted = false;

    public static float playerOneLeftTriggerThreshold = 0.0f;

    public static float playerOneRightTriggerThreshold = 0.0f;

    public static float playerTwoLeftTriggerThreshold = 0.0f;

    public static float playerTwoRightTriggerThreshold = 0.0f;
}
