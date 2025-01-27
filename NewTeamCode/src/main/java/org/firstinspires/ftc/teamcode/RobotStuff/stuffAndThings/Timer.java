package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Timer {

    public final long startTimeNano;
    public final long endTimeNano;

    public Timer(double durationSeconds) {
        startTimeNano = System.nanoTime();
        endTimeNano = startTimeNano + (long) (ElapsedTime.SECOND_IN_NANO * durationSeconds);
    }

    public boolean done() {
        return System.nanoTime() > endTimeNano;
    }
}
