package org.firstinspires.ftc.teamcode.pedroPathing;

public class TimeStamp {

    Runnable code;
    double time;
    int path;
    boolean beenRun = false;

    public TimeStamp (Runnable code, double time, int path) {
        this.code = code;
        this.time = time;
        this.path = path;
    }

    public void setCode(Runnable code) {
        this.code = code;
    }

    public void setTime(double time) {
        this.time = time;
    }

    public void setPath(int path) {
        this.path = path;
    }

    public Runnable getCode() {
        return code;
    }

    public double getTime() {
        return time;
    }

    public int getPath() {
        return path;
    }

    public void run() {
        code.run();
        beenRun = true;
    }

    public boolean hasBeenRun() {
        return beenRun;
    }

    public void reset() {
        beenRun = false;
    }
}
