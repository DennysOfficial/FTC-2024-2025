package org.firstinspires.ftc.teamcode.TestingOpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftPivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.RightLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.RightPivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveIntakeMotor;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveIntakeServo;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.PassiveGrabber;
import org.firstinspires.ftc.teamcode.pedroPathing.Automous;
import org.firstinspires.ftc.teamcode.pedroPathing.AutomousNoLift;
import org.firstinspires.ftc.teamcode.pedroPathing.LiftTimeStamp;
import org.firstinspires.ftc.teamcode.pedroPathing.TimeStamp;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.SingleRunAction;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.List;

@Autonomous(name = "SoupcOpMode_0-5-Z 2.0.0")
public class Auto_Test_05Z extends OpMode{

    List<LynxModule> allHubs;

    private final Pose startPose = new Pose(9,64.5, Math.toRadians(0));  // This is where the robot starts

    //Points of Interest
    Point rungPoint1 = new Point(39, 64.5, Point.CARTESIAN);
    Point rungPoint2 = new Point(39, 66, Point.CARTESIAN);
    Point rungPoint3 = new Point(39, 67.5, Point.CARTESIAN);
    Point rungPoint4 = new Point(39, 69, Point.CARTESIAN);
    Point rungPoint5 = new Point(39, 70.5, Point.CARTESIAN);

    Point pickupPoint1 = new Point(9, 12, Point.CARTESIAN);
    Point pickupPoint2 = new Point(10.5, 36, Point.CARTESIAN);
    Point pickupPoint3 = new Point(9, 36, Point.CARTESIAN);

    Point samplePoint1 = new Point(24, 24, Point.CARTESIAN);
    Point samplePoint2 = new Point(24, 12, Point.CARTESIAN);

    Point parkPoint = new Point(24, 48, Point.CARTESIAN);


    public PathChain movement1, movement2, movement3, movement4;

    public static double liftPosSpike1 = 0;
    public static double liftPosSpike3 = 0;

    // Other misc. stuff
    private Follower follower;

    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;

    LeftLift leftLift;
    LeftPivot leftPivot;
    ActiveIntakeMotor intake;

    RightLift rightLift;
    RightPivot rightPivot;
    PassiveGrabber grabber;

    RobotConfig config;

    Automous automous; //CHANGE THIS WHEN NEW ROBOT

    int listPointer = 1;

    private Timer pathTimer;

    Runnable r1, r2;
    SingleRunAction a1, a2;

    @Override
    public void init() {

        pathTimer = new Timer();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk


        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        config = new RobotConfig(this);

        leftLift = new LeftLift(ControlAxis.ControlMode.positionControl,this, config);
        leftPivot = new LeftPivot(ControlAxis.ControlMode.positionControl,this, config);
        intake = new ActiveIntakeMotor(this, config);

        rightLift = new RightLift(ControlAxis.ControlMode.positionControl,this, config);
        rightPivot = new RightPivot(ControlAxis.ControlMode.positionControl,this, config);
        grabber = new PassiveGrabber(this, config, leftLift, leftPivot);

        leftLift.assignPivot(leftPivot);
        leftPivot.assignLift(leftLift);
        rightLift.assignPivot(rightPivot);
        rightPivot.assignLift(rightLift);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);



        automous = new Automous(this, leftLift, leftPivot, rightLift, rightPivot, intake, grabber, config, follower);
    }



    public void buildPaths() {
        movement1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(startPose), rungPoint1)))
                .addTemporalCallback(0.5, () -> {
                    grabber.Score();
                })
                .addPath(new Path(new BezierLine(rungPoint1, samplePoint1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addTemporalCallback(1, () -> {
                    grabber.Rest();
                })
                .addTemporalCallback(2, () -> {
                    rightPivot.setTargetPosition(90);
                    rightLift.setTargetPosition(liftPosSpike1);
                })
                .build();
        movement2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(samplePoint1,samplePoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addTemporalCallback(0.5, () -> {
                    rightPivot.setTargetPosition(90);
                    rightLift.setTargetPosition(liftPosSpike1);
                })
                .build();
        movement3 = follower.pathBuilder()
                .addPath(new Path(new BezierPoint(samplePoint2)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(340))
                .addTemporalCallback(0.5, () -> {
                    rightPivot.setTargetPosition(90);
                    rightLift.setTargetPosition(liftPosSpike3);
                })
                .build();
        movement4 = follower.pathBuilder()
                .addPath(new Path(new BezierPoint(samplePoint2)))
                .setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(0))
                .addTemporalCallback(0, () -> {
                    grabber.Collect();
                })
                .addPath(new Path(new BezierLine(samplePoint2, pickupPoint1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new Path(new BezierLine(pickupPoint1, rungPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addTemporalCallback(0.5, ()-> {
                    grabber.Score();
                })
                .addPath(new Path(new BezierLine(rungPoint2, pickupPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addTemporalCallback(0.5, () -> {
                    grabber.Collect();
                })
                .addPath(new Path(new BezierLine(pickupPoint2, pickupPoint3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new Path(new BezierLine(pickupPoint2, rungPoint3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addTemporalCallback(0.5, ()-> {
                    grabber.Score();
                })
                .addPath(new Path(new BezierLine(rungPoint3, pickupPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addTemporalCallback(0.5, () -> {
                    grabber.Collect();
                })
                .addPath(new Path(new BezierLine(pickupPoint2, pickupPoint3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new Path(new BezierLine(pickupPoint2, rungPoint4)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addTemporalCallback(0.5, ()-> {
                    grabber.Score();
                })
                .addPath(new Path(new BezierLine(rungPoint4, pickupPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addTemporalCallback(0.5, () -> {
                    grabber.Collect();
                })
                .addPath(new Path(new BezierLine(pickupPoint2, pickupPoint3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new Path(new BezierLine(pickupPoint2, rungPoint5)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addTemporalCallback(0.5, ()-> {
                    grabber.Score();
                })
                .addPath(new Path(new BezierLine(rungPoint5, parkPoint)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(240))
                .addTemporalCallback(0.5, () -> {
                    grabber.Collect();
                })
                .addTemporalCallback(1, () -> {
                    rightPivot.setTargetPosition(90);
                    rightLift.setTargetPosition(0);
                    intake.moveWrist(0.2);
                })
                .build();

        r1 = () -> {
            rightLift.setTargetPosition(0);
            rightPivot.setTargetPosition(-69);
            intake.moveWrist(0.85);
        };

        a1 = new SingleRunAction(r1);

        r2 = () -> {
            intake.intakeForDuration(1);
        };

        a2 = new SingleRunAction(r2);
    }

    public void routine() {
        switch (listPointer) {
            case 1:

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= 10) {
                    listPointer = 2;
                    pathTimer.resetTimer();
                    intake.moveWrist(0.2);
                    intake.intakeForDuration(1);
                }

                break;
            case 2:

                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    a1.run();
                }

                if (pathTimer.getElapsedTimeSeconds() >= 2) {
                    a2.run();
                }

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= 3) {
                    listPointer = 3;
                    follower.followPath(movement2);
                    pathTimer.resetTimer();
                    a1.reset();
                    a2.reset();
                }

                break;
            case 3:

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= 10) {
                    listPointer = 4;
                    pathTimer.resetTimer();
                    intake.moveWrist(0.2);
                    intake.intakeForDuration(1);
                }

                break;
            case 4:

                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    a1.run();
                }

                if (pathTimer.getElapsedTimeSeconds() >= 2) {
                    a2.run();
                }

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= 3) {
                    listPointer = 5;
                    follower.followPath(movement3);
                    pathTimer.resetTimer();
                    a1.reset();
                    a2.reset();
                }

                break;
            case 5:

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= 10) {
                    listPointer = 6;
                    pathTimer.resetTimer();
                    intake.moveWrist(0.2);
                    intake.intakeForDuration(1);
                }

                break;
            case 6:

                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    a1.run();
                }

                if (pathTimer.getElapsedTimeSeconds() >= 2) {
                    a2.run();
                }

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= 3) {
                    listPointer = 7;
                    follower.followPath(movement4);
                    pathTimer.resetTimer();
                    a1.reset();
                    a2.reset();
                }
                break;
            case 7:
                break;
        }
    }

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        deltaTime = frameTimer.milliseconds();
        frameTimer.reset();

        routine();
        follower.update();
        leftLift.update();
        leftPivot.update();
        rightLift.update();
        rightPivot.update();
        intake.update();

        telemetry.addData("path state", automous.getPath());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("deltaTime", deltaTime);
        telemetry.addData("waitTime", time);
        telemetry.update();
    }

    @Override
    public void start() {
        buildPaths();
        follower.followPath(movement1);
        pathTimer.resetTimer();
        rightLift.setTargetPosition(0);
        rightPivot.setTargetPosition(-69);

        deltaTime = 0;
        frameTimer.reset();
        // Put first path instead of brackets
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }
}

/**
 * 8==D
 */