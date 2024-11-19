
package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories;


import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.MathStuff;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ReadOnlyRuntime;

public class LinearTrajectory implements Trajectory {

    public final double startTime;
    public final double duration;
    public final double endTime;

    public double startPosition;
    public double endPosition;

    ReadOnlyRuntime runtime;

    public LinearTrajectory(ReadOnlyRuntime runtime, double startPosition, double endPosition, double durationSeconds) {
        this.runtime = runtime;

        this.duration = durationSeconds;

        startTime = runtime.seconds();
        endTime = startTime + durationSeconds;

        this.startPosition = startPosition;
        this.endPosition = endPosition;
    }


    public MotionState sampleTrajectory() {
        MotionState motionState = new MotionState();

        double interpolationAmount = (runtime.seconds() - startTime) / duration;

        motionState.position = MathStuff.lerp(startPosition, endPosition, interpolationAmount);
        motionState.velocity = (endPosition - startPosition) / duration;

        return motionState;
    }

    public boolean isActive() {
        return (runtime.seconds() >= startTime && runtime.seconds() < endTime);
    }
}
