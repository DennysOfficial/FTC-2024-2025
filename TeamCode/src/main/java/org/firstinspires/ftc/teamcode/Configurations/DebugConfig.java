package org.firstinspires.ftc.teamcode.Configurations;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DebugConfig {

    float makeTheSquigglyThingGoAwayPlease = 420; // very important
    public static boolean pivotPositionAndDerivatives = false;
    public boolean isPivotPositionAndDerivativesDebug() {
        return pivotPositionAndDerivatives;
    }
}
