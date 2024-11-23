package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StopWatch {

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
        return (double) (startTimeNano - System.nanoTime()) / ElapsedTime.MILLIS_IN_NANO;
    }

    public double getTimeSeconds() {
        return (double) (startTimeNano - System.nanoTime()) / ElapsedTime.SECOND_IN_NANO;
    }

    public void addTimeToTelemetryAndReset(Telemetry telemetry, String timeName) {
        telemetry.addData(timeName, getTimeMilli());
        reset();
    }
}
