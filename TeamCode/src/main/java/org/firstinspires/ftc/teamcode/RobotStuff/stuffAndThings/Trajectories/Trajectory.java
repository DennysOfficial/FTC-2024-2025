package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories;

public interface Trajectory {
    MotionState sampleTrajectory();
    boolean isActive();
}
