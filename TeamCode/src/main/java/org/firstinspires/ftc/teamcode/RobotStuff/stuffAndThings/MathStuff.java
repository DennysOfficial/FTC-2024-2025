package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings;

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

    public static long map(long x, long in_min, long in_max, long out_min, long out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public static double map(double x, double in_min, double in_max, double out_min, double out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
