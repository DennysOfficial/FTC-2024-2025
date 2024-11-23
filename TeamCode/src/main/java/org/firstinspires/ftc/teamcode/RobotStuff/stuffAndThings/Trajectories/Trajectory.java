package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Trajectory {
    public abstract MotionState sampleTrajectory();

    public abstract boolean isActive();

    double getTimeSeconds() {
        return System.nanoTime() / ((double) ElapsedTime.SECOND_IN_NANO);
    }
}
