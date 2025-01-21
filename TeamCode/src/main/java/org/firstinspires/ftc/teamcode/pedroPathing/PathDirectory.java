package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public class PathDirectory {

    private double PPA;
    private double LPA;

    private PathChain path;

    private int armPosSpecimen;

    private double timeout;

    public PathDirectory(double pivotPosSample, double liftPosSample, PathChain path, int armPosSpecimen, double timeout) {
        setPPA(pivotPosSample);
        setLPA(liftPosSample);
        setPath(path);
        setArmPosSpecimen(armPosSpecimen);
        setTimeout(timeout);
    }

    public void setPPA(double PPA) {
        this.PPA = PPA;
    }

    public void setLPA(double LPA) {
        this.LPA = LPA;
    }

    public void setPath(PathChain path) {
        this.path = path;
    }

    public void setTimeout(double timeout) {
        this.timeout = timeout;
    }

    public void setArmPosSpecimen(int armPosSpecimen) {
        this.armPosSpecimen = armPosSpecimen;
    }

    public double getPPA() {
        return PPA;
    }

    public double getLPA() {
        return LPA;
    }

    public PathChain getPath() {
        return path;
    }

    public int getArmPosSpecimen() {
        return armPosSpecimen;
    }

    public double getTimeout() {
        return timeout;
    }
}
