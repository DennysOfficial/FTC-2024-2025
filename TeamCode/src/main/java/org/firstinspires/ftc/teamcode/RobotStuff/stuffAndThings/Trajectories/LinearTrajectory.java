
package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories;


import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.MathStuff;

public class LinearTrajectory extends Trajectory {

    public final double startTime;
    public final double duration;
    public final double endTime;

    public double startPosition;
    public double endPosition;


    public LinearTrajectory(double startPosition, double endPosition, double durationSeconds) {

        this.duration = durationSeconds;

        startTime = getTimeSeconds();
        endTime = startTime + durationSeconds;

        this.startPosition = startPosition;
        this.endPosition = endPosition;
    }



    public MotionState sampleTrajectory() {
        MotionState motionState = new MotionState();

        double interpolationAmount = (getTimeSeconds() - startTime) / duration;

        motionState.position = MathStuff.lerp(startPosition, endPosition, interpolationAmount);
        motionState.velocity = (endPosition - startPosition) / duration;

        return motionState;
    }

    public boolean isActive() {
        return (getTimeSeconds() >= startTime && getTimeSeconds() < endTime);
    }
}
