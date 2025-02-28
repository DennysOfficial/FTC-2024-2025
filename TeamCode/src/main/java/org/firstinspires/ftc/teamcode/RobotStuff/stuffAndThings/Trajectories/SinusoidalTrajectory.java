
package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories;


public class SinusoidalTrajectory extends Trajectory {

    public final double startTime;
    public final double duration;
    public final double endTime;

    public double startPosition;
    public double endPosition;



    public SinusoidalTrajectory(double startPosition, double endPosition, double durationSeconds) {

        this.duration = durationSeconds;

        startTime = getTimeSeconds();
        endTime = startTime + durationSeconds;

        this.startPosition = startPosition;
        this.endPosition = endPosition;
    }


    public MotionState sampleTrajectory() {
        MotionState motionState = new MotionState();

        double theta = (getTimeSeconds() - startTime) / duration * Math.PI;
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
        return (getTimeSeconds() >= startTime && getTimeSeconds() < endTime);
    }
}
