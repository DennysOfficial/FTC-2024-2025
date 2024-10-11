package org.firstinspires.ftc.teamcode.Configurations;

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
}
