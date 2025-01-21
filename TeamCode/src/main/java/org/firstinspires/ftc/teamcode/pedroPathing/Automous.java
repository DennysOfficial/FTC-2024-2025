package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftPivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.RightLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.RightPivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveIntakeMotor;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveIntakeServo;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.PassiveGrabber;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.ArrayList;


public class Automous{

    private Follower follower;

    LeftLift leftLift;
    LeftPivot leftPivot;

    RightLift rightLift;
    RightPivot rightPivot;

    ActiveIntakeMotor intake;
    PassiveGrabber grabber;

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
    public Automous(OpMode opmode, LeftLift leftLift, LeftPivot leftPivot, RightLift rightLift, RightPivot rightPivot, ActiveIntakeMotor activeIntake, PassiveGrabber grabber, RobotConfig robotConfig, Follower follower) {
        currentOpMode = opmode;
        this.leftLift = leftLift;
        this.leftPivot = leftPivot;
        this.rightLift = rightLift;
        this.rightPivot = rightPivot;
        intake = activeIntake;
        config = robotConfig;
        this.follower = follower;
    }

    /**
     * Add a path to storage, to be run later.
     * @param pivotPosSample Pivot position at the end of the path. Note that the pivot will only start moving to this position at the end of the path and may take some time to reach it.
     * @param liftPosSample Lift position at the end of the path. Note that the lift will only start moving to this position at the end of the path and may take some time to reach it.
     * @param path The path for the robot to follow.
     * @param armPosSpecimen 0 = Rest, 1 = Collect, 2 = Score.
     * @param timeout Time in seconds to move on to the next path. It is a failsafe to ensure that the rest of the routine is not compromised upon failure to reach the target point.
     */
    public void addPath(double pivotPosSample, double liftPosSample, PathChain path, int armPosSpecimen, double timeout) {
        pathDirectory.add(new PathDirectory(pivotPosSample, liftPosSample, path, armPosSpecimen, timeout));
    }

    //TODO: Resolve placeholder comments
    public void routine() {
        currentPath = pathDirectory.get(listPointer);

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
                if (current.usesSpecimenLift()) {
                    switch (currentPath.getArmPosSpecimen()) {
                        case 0:
                            rightLift.setTargetPosition(0);
                            rightPivot.setTargetPosition(0);
                            grabber.Rest();
                            break;
                        case 1:
                            rightLift.setTargetPosition(0);
                            rightPivot.setTargetPosition(0);
                            grabber.Collect();
                            break;
                        case 2:
                            rightLift.setTargetPosition(0);
                            rightPivot.setTargetPosition(0);
                            grabber.Score();
                            break;
                    }
                } else {
                    leftLift.setTargetPosition(current.getLiftPos());
                    leftPivot.setTargetPosition(current.getPivotPos());
                }
            }
        }

        if (currentPath.getPath() != null && follower.atParametricEnd() || currentPath.getTimeout() <= pathTimer.getElapsedTimeSeconds()) {
            leftLift.setTargetPosition(currentPath.getPPA());
            leftPivot.setTargetPosition(currentPath.getLPA());
            //Specimen lift code go brr
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
        leftLift.update();
        leftPivot.update();
        intake.update();
    }

    public void resetTimer() {
        pathTimer.resetTimer();
    }
}

/**
 * 8==D
 */