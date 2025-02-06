package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings;

public class Ramp {
    public double ratePerSecond;
    double previousTarget;

    public Ramp(double startingValue, double ratePerSecond) {
        this.previousTarget = startingValue;
        this.ratePerSecond = ratePerSecond;
    }

    double deltaValue;
    double targetDeltaValue;
    double rampedTargetValue;

    public double getRampedValue(double targetValue, double deltaTime) {
        deltaValue = targetValue - previousTarget;
        targetDeltaValue = Math.copySign(ratePerSecond * deltaTime, deltaValue);
        if (Math.abs(deltaValue)  - Math.abs(targetDeltaValue) < 0)
            rampedTargetValue = targetValue;
        else
            rampedTargetValue = previousTarget + targetDeltaValue;
        return previousTarget = rampedTargetValue;
    }
}
