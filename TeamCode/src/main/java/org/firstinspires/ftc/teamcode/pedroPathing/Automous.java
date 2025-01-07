package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Lift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Pivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveIntake;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ReadOnlyRuntime;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.ArrayList;


public class Automous{

    private Follower follower;

    private final ReadOnlyRuntime runtime = new ReadOnlyRuntime();
    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;

    Lift lift;
    Pivot pivot;
    ActiveIntake intake;
    RobotConfig config;

    ArrayList<PathDirectory> pathDirectory = new ArrayList<PathDirectory>(0);

    ArrayList<TimeStamp> timeStamps = new ArrayList<TimeStamp>(0);

    ArrayList<LiftTimeStamp> liftTimeStamps = new ArrayList<LiftTimeStamp>(0);

    OpMode currentOpMode;
    PathDirectory currentPath;
    int listPointer = 0;

    Timer pathTimer;

    /**
     * A class for easily storing and executing autonomous routines.
     */
    public Automous(OpMode opmode, Lift lift, Pivot pivot, ActiveIntake activeIntake, RobotConfig robotConfig, Follower follower) {
        currentOpMode = opmode;
        this.lift = lift;
        this.pivot = pivot;
        intake = activeIntake;
        config = robotConfig;
        this.follower = follower;
    }

    /**
     * Add a path to storage, to be run later.
     * @param pivotPosStart Pivot position at the beginning of the path. Note that the pivot will only start moving to this position at the start of the path and may take some time to reach it.
     * @param liftPosStart Lift position at the beginning of the path. Note that the lift will only start moving to this position at the start of the path and may take some time to reach it.
     * @param path The path for the robot to follow.
     * @param pivotPosEnd Pivot position at the end of the path. Note that the pivot will only start moving to this position at the end of the path and may take some time to reach it.
     * @param liftPosEnd Lift position at the end of the path. Note that the lift will only start moving to this position at the end of the path and may take some time to reach it.
     * @param timeout Time in seconds to move on to the next path. It is a failsafe to ensure that the rest of the routine is not compromised upon failure to reach the target point.
     */
    public void addPath(double pivotPosStart, double liftPosStart, PathChain path, double pivotPosEnd, double liftPosEnd, double timeout) {
        pathDirectory.add(new PathDirectory(pivotPosStart, liftPosStart, path, pivotPosEnd, liftPosEnd, timeout));
    }

    public void routine() {
        currentPath = pathDirectory.get(listPointer);

        pivot.setTargetPosition(currentPath.getPPS());
        lift.setTargetPosition(currentPath.getLPS());

        if (currentPath.getPath() != null) {
            follower.followPath(currentPath.getPath());
        }

        for (TimeStamp current : timeStamps) {
            if (current.getTime() <= pathTimer.getElapsedTimeSeconds() && !current.hasBeenRun() && current.getPath() == listPointer + 1) {
                current.run();
            }
        }

        for (LiftTimeStamp current : liftTimeStamps) {
            if (current.getTime() <= pathTimer.getElapsedTimeSeconds() && !current.hasBeenRun() && current.getPath() == listPointer + 1) {
                currentPath.setPSS(current.getPivotPos());
                currentPath.setLSS(current.getLiftPos());
            }
        }

        if (currentPath.getPath() != null && follower.atParametricEnd() || currentPath.getTimeout() <= pathTimer.getElapsedTimeSeconds()) {
            pivot.setTargetPosition(currentPath.getPPE());
            lift.setTargetPosition(currentPath.getLPE());
            listPointer = listPointer + 1;
            pathTimer.resetTimer();
            if (listPointer >= pathDirectory.size()){
                currentOpMode.stop();
            }
        }
    }

    public void addTimeStamp(TimeStamp timeStamp) {
        timeStamps.add(timeStamp);
    }

    public void addLiftTimeStamp(LiftTimeStamp liftTimeStamp) {
        liftTimeStamps.add(liftTimeStamp);
    }

    public int getPath() {
        return listPointer + 1;
    }

    public void update() {
        follower.update();
        lift.update();
        pivot.update();
        intake.update();
    }

    public void resetTimer() {
        pathTimer.resetTimer();
    }
}

/**
 * 8==D
 */