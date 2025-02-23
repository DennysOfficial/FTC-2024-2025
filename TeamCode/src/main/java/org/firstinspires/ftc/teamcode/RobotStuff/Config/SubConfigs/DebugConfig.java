package org.firstinspires.ftc.teamcode.RobotStuff.Config.SubConfigs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DebugConfig {

    public static boolean allPositionDebug = true;

    public boolean getAllPositionDebug() {
        return allPositionDebug;
    }


    public static boolean positionDerivatives = false;

    public boolean positionDerivativesDebug() {
        return positionDerivatives;
    }


    public static boolean motorPowerDebug = false;

    public boolean getMotorPowerDebug() {
        return motorPowerDebug;
    }


    public static boolean inputDebug = false;

    public boolean inputDebug() {
        return inputDebug;
    }


    public static boolean PIDDebug = false;

    public boolean getPIDDebug() {
        return PIDDebug;
    }


    public static boolean controlModeDebug = false;

    public boolean getStateDebug() {
        return controlModeDebug;
    }


    public static boolean timeBreakdownDebug = false;

    public boolean getTimeBreakdownDebug() {
        return timeBreakdownDebug;
    }

}
