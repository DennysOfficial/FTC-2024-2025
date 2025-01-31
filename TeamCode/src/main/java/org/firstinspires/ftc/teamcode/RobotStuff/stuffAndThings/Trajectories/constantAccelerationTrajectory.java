
package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories;


import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.MathStuff;

public class constantAccelerationTrajectory extends Trajectory {


    public MotionState startMotionState;
    public MotionState endMotionState;

    double maxAcceleration;
    double accelerationDistance;
    double maxSpeed;
    double maxSpeedDistance;
    double maxDeceleration;
    double decelerationDistance;
    double travelDirection;

    TrajectoryPhase trajectoryPhase = TrajectoryPhase.acceleration;

    enum TrajectoryPhase {
        acceleration,
        constantVelocity,
        deceleration,
        done,
    }


    public constantAccelerationTrajectory(MotionState startMotionState, MotionState endMotionState, double maxAcceleration, double maxSpeed, double maxDeceleration) {

        this.maxAcceleration = maxAcceleration;
        this.maxSpeed = maxSpeed;
        this.maxDeceleration = maxDeceleration;

        this.startMotionState = startMotionState;
        this.endMotionState = endMotionState;

        lastTargetMotionState = startMotionState;


        travelDirection = endMotionState.position - startMotionState.position;

        //double deltaVelocity = maxSpeed - Math.abs(startMotionState.velocity);
        double thing = Math.pow(Math.copySign(maxSpeed, travelDirection),2) - Math.pow(startMotionState.velocity,2);
        double topSpeedAccelerationDistance =  thing/ (2.0 * maxAcceleration);
    }

    MotionState lastTargetMotionState;

    public MotionState sampleTrajectory() {
        updateTimeSinceStart();
        MotionState targetMotionState = new MotionState();

        switch (trajectoryPhase) {
            case acceleration:
                if (Math.abs(lastTargetMotionState.velocity) < maxSpeed && Math.abs(getStoppingDistance()) < distanceToEnd()) {
                    targetMotionState.position = startMotionState.position + startMotionState.velocity * timeSinceStart + 0.5 * maxAcceleration * timeSinceStart * timeSinceStart;
                    break;
                }
                if (Math.abs(getStoppingDistance()) >= distanceToEnd())
                    trajectoryPhase = TrajectoryPhase.deceleration;
                else if (Math.abs(lastTargetMotionState.velocity) >= maxSpeed)
                    trajectoryPhase = TrajectoryPhase.constantVelocity;

            case constantVelocity:

                if (Math.abs(getStoppingDistance()) >= distanceToEnd())
                    trajectoryPhase = TrajectoryPhase.deceleration;
            case deceleration:
                break;

        }


        return targetMotionState;
    }

    double getStoppingDistance() {
        return (endMotionState.velocity - lastTargetMotionState.velocity) / (2 * maxDeceleration);
    }

    double distanceToEnd() {
        return lastTargetMotionState.position - endMotionState.position;
    }

    public boolean isActive() {
        return (trajectoryPhase == TrajectoryPhase.done);
    }
}
