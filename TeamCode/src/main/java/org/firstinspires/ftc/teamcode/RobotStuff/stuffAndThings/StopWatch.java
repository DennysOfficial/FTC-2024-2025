package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StopWatch {

    public boolean debug = true;

    public long startTimeNano;

    public StopWatch() {
        startTimeNano = System.nanoTime();

    }

    public void reset() {
        startTimeNano = System.nanoTime();
    }

    public long getTimeNano() {
        return startTimeNano - System.nanoTime();
    }

    public double getTimeMilli() {
        return (startTimeNano - System.nanoTime()) / (double) ElapsedTime.MILLIS_IN_NANO;
    }

    public double getTimeSeconds() {
        return (startTimeNano - System.nanoTime()) / (double) ElapsedTime.SECOND_IN_NANO;
    }

    public void addTimeToTelemetryAndReset(Telemetry telemetry, String timeName) {
        if(!debug)
            return;
        telemetry.addData(timeName, getTimeMilli());
        reset();
    }
}
