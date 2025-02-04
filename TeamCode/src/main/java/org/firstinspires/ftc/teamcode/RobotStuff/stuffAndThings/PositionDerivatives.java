package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings;

public class PositionDerivatives {
    double velocity = 0;

    public double getVelocity() {
        return velocity;
    }


    double acceleration = 0;

    public double getAcceleration() {
        return acceleration;
    }

    double previousPosition;
    double previousVelocity = 0;
    public PositionDerivatives(double startingPosition) {
        previousPosition = startingPosition;
    }

    public void update(double position, double deltaTime) {
        updateVelocity(position, deltaTime);
        updateAcceleration(deltaTime);
    }

    void updateVelocity(double position, double deltaTime) {
        velocity = -(previousPosition - (previousPosition = position)) / deltaTime;
    }

    void updateAcceleration(double deltaTime) {
        acceleration = -(previousVelocity - (previousVelocity = velocity)) / deltaTime;
    }

}
