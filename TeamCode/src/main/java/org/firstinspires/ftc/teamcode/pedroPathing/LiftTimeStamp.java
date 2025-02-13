package org.firstinspires.ftc.teamcode.pedroPathing;

public class LiftTimeStamp {

    double liftPos, pivotPos;
    double time;
    int path;
    boolean beenRun = false;

    public LiftTimeStamp(double pivotPos, double liftPos, double time, int path) {
        this.liftPos = liftPos;
        this.pivotPos = pivotPos;
        this.time = time;
        this.path = path;
    }

    public void setLiftPos(double liftPos) {
        this.liftPos = liftPos;
    }

    public void setPivotPos(double pivotPos) {
        this.pivotPos = pivotPos;
    }

    public void setTime(double time) {
        this.time = time;
    }

    public void setPath(int path) {
        this.path = path;
    }

    public double getLiftPos() {
        return liftPos;
    }

    public double getPivotPos() {
        return pivotPos;
    }

    public double getTime() {
        return time;
    }

    public int getPath() {
        return path;
    }

    public boolean hasBeenRun() {
        return beenRun;
    }

    public void reset() {
        beenRun = false;
    }
}
