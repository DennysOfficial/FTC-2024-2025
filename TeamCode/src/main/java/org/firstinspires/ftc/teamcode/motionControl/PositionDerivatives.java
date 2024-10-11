package org.firstinspires.ftc.teamcode.motionControl;

public class PositionDerivatives {
    double velocity = 0;

    public double getVelocity() {
        return velocity;
    }


    double acceleration = 0;

    public double getAcceleration() {
        return acceleration;
    }


    double jerk = 0;

    public double getJerk() {
        return jerk;
    }


    double previousAngle;
    double previousVelocity = 0;
    double previousAcceleration = 0;

    public PositionDerivatives(double startingPosition) {
        previousAngle = startingPosition;
    }

    public void update(double position, double deltaTime) {
        updateVelocity(position, deltaTime);
        updateAcceleration(deltaTime);
        updateJerk(deltaTime);
    }

    void updateVelocity(double position, double deltaTime) {
        velocity = -(previousAngle - (previousAngle = position)) / deltaTime;
    }

    void updateAcceleration(double deltaTime) {
        acceleration = -(previousVelocity - (previousVelocity = velocity)) / deltaTime;
    }

    void updateJerk(double deltaTime) {
        jerk = -(previousAcceleration - (previousAcceleration = acceleration)) / deltaTime;
    }
}
