package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ReadOnlyRuntime {
    public final ElapsedTime elapsedTime;

    boolean initialized = false;


    public void start() {
        if (initialized)
            throw new Error("read only runtime can only be started once. that's like the whole point");

        elapsedTime.startTime();
        initialized = true;
    }

    public ReadOnlyRuntime(ElapsedTime elapsedTime) {
        this.elapsedTime = elapsedTime;
    }

    public double time(){
        return elapsedTime.time();
    }
    public double seconds(){
        return elapsedTime.seconds();
    }
    public double milliseconds(){
        return elapsedTime.milliseconds();
    }
    public double nanoseconds(){
        return elapsedTime.nanoseconds();
    }
}
