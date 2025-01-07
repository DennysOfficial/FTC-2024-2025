package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class PathDirectory {

    private double PPS;
    private double LPS;

    private PathChain path;

    private double PPE;
    private double LPE;

    private double timeout;

    public PathDirectory(double pivotPosStart, double liftPosStart, PathChain path, double pivotPosEnd, double liftPosEnd, double timeout) {
        setPSS(pivotPosStart);
        setLSS(liftPosStart);
        setPath(path);
        setPSE(pivotPosEnd);
        setLSE(liftPosEnd);
        setTimeout(timeout);
    }

    public void setPSS(double PPS) {
        this.PPS = PPS;
    }

    public void setLSS(double LPS) {
        this.LPS = LPS;
    }

    public void setPath(PathChain path) {
        this.path = path;
    }

    public void setPSE(double PSE) {
        this.PPE = PSE;
    }

    public void setLSE(double LSE) {
        this.LPE = LSE;
    }

    public void setTimeout(double timeout) {
        this.timeout = timeout;
    }

    public double getPPS() {
        return PPS;
    }

    public double getLPS() {
        return LPS;
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
