package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public class PathDirectory {

    private double PPA;
    private double LPA;

    private PathChain path;

    private double PPE;
    private double LPE;

    private double timeout;

    public PathDirectory(double pivotPosSample, double liftPosSample, PathChain path, double pivotPosSpecimen, double liftPosSpecimen, double timeout) {
        setPPA(pivotPosSample);
        setLPA(liftPosSample);
        setPath(path);
        setPPE(pivotPosSpecimen);
        setLPE(liftPosSpecimen);
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

    public void setPPE(double PPE) {
        this.PPE = PPE;
    }

    public void setLPE(double LPE) {
        this.LPE = LPE;
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

    public double getPPE() {
        return PPE;
    }

    public double getLPE() {
        return LPE;
    }

    public double getTimeout() {
        return timeout;
    }
}
