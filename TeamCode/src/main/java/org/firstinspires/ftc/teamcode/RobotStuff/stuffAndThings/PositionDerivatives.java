package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings;

import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories.MotionState;

public class PositionDerivatives {

    public MotionState motionState;
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
    public PositionDerivatives(double startingPosition, MotionState motionState) {
        this.motionState = motionState;
        previousPosition = startingPosition;
    }
    public PositionDerivatives(double startingPosition) {
        motionState = new MotionState();
        motionState.position = startingPosition;
        previousPosition = startingPosition;
    }


    public void  update(double position, double deltaTime) {
        motionState.position = position;
        updateVelocity(deltaTime);
        updateAcceleration(deltaTime);
    }

    void updateVelocity(double deltaTime) {
        motionState.velocity = -(previousPosition - (previousPosition = motionState.position)) / deltaTime;
    }

    void updateAcceleration(double deltaTime) {
        motionState.acceleration = -(previousVelocity - (previousVelocity = motionState.velocity)) / deltaTime;
    }

}
