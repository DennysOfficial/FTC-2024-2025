package org.firstinspires.ftc.teamcode.Config.SubConfigs;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DebugConfig {

    float makeTheSquigglyThingGoAwayPlease = 420; // very important
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
}
