package org.firstinspires.ftc.teamcode;

public class MathStuff {

    /**
     * linear interpolate between two values
     * @param interpolationAmount range from 0 - 1. 0 returns value1 and 1 returns value2
     * @return interpolated value
     */
    public static double lerp(double value1, double value2, double interpolationAmount) {
        double valueDelta = value2 - value1;
        return value1 + valueDelta * interpolationAmount;
    }
}
