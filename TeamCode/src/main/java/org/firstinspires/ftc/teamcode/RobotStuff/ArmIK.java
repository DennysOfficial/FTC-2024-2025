package org.firstinspires.ftc.teamcode.RobotStuff;

public class ArmIK {
    double extensionDistance;
    double extensionAxisOffset;
    double targetHeightOffset;


    void updateCoefficients() {
        updateA();
        updateB();
        updateC();
    }

    private double a;

    void updateA() {
        a = extensionDistance * extensionDistance + extensionAxisOffset * extensionAxisOffset;
    }

    private double b;

    void updateB() {
        b = 2 * extensionAxisOffset * targetHeightOffset;
    }

    private double c;

    void updateC() {
        c = targetHeightOffset * targetHeightOffset - extensionDistance * extensionDistance;
    }

    double getSinTheta() {
        return (-b + Math.sqrt(b * b - 4 * a * c)) / 2 * a;
    }

    public double getTargetAngle(double extensionDistanced, double extensionAxisOffset, double targetHeightOffset) {
        this.extensionDistance = extensionDistanced;
        this.extensionAxisOffset = extensionAxisOffset;
        this.targetHeightOffset = targetHeightOffset;

        updateCoefficients();

        return 180 - Math.asin(getSinTheta());
    }
}
