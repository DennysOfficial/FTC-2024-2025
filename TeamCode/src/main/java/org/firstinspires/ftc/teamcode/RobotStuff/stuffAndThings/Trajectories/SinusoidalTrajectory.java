
package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.MathStuff;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ReadOnlyRuntime;

public class SinusoidalTrajectory implements Trajectory {

    public final double startTime;
    public final double duration;
    public final double endTime;

    public double startPosition;
    public double endPosition;

    ReadOnlyRuntime runtime;

    public SinusoidalTrajectory(ReadOnlyRuntime runtime, double startPosition, double endPosition, double durationSeconds) {
        this.runtime = runtime;

        this.duration = durationSeconds;

        startTime = runtime.seconds();
        endTime = startTime + durationSeconds;

        this.startPosition = startPosition;
        this.endPosition = endPosition;
    }


    public MotionState sampleTrajectory() {
        MotionState motionState = new MotionState();

        double theta = (runtime.seconds() - startTime) / duration * Math.PI;
        //telemetry.addData("theta", theta);
        double interpolationAmount = 0.5 - Math.cos(theta) / 2;
        //telemetry.addData("interpolation amount", interpolationAmount);
        double distance = endPosition - startPosition;

        motionState.position = startPosition + interpolationAmount * distance;
        motionState.velocity = distance / 2 * Math.sin(theta);
        motionState.acceleration = distance / 2 * Math.cos(theta);

        return motionState;
    }

    public boolean isActive() {
        return (runtime.seconds() >= startTime && runtime.seconds() < endTime);
    }
}
