package org.firstinspires.ftc.teamcode;

public class MathStuff {

    public static double lerp(double point1, double point2, double interpolationAmount) {
        double pointDelta = point2 - point1;
        return point1 + pointDelta * interpolationAmount;
    }
}
