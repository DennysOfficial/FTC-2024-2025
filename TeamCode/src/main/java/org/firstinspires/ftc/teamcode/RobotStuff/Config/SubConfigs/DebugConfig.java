package org.firstinspires.ftc.teamcode.RobotStuff.Config.SubConfigs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DebugConfig {

    public static boolean allPositionDebug = false;

    public boolean getAllPositionDebug() {
        return allPositionDebug;
    }

    public static boolean pivotPositionAndDerivatives = false;

    public boolean pivotPositionAndDerivativesDebug() {
        return pivotPositionAndDerivatives;
    }

    public static boolean pivotTorqueDebug = false;

    public boolean pivotTorqueDebug() {
        return pivotTorqueDebug;
    }

    public static boolean liftPositionAndDerivatives = false;

    public boolean liftPositionAndDerivativesDebug() {
        return liftPositionAndDerivatives;
    }

    public static boolean liftTorqueDebug = false;

    public boolean liftTorqueDebug() {
        return liftTorqueDebug;
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

    public boolean getControlModeDebug() {
        return controlModeDebug;
    }

    public static boolean timeBreakdownDebug = true;

    public boolean getTimeBreakdownDebug() {
        return timeBreakdownDebug;
    }

}
