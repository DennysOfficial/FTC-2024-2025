package org.firstinspires.ftc.teamcode.Autonomous_OpModes;


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
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ClawAndStuff;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.List;

@Autonomous(name = "Superman: SoupcOpMode_0-5-Z 0.0.1", group = "SoupcOpModes")
public class Auto_Test_05Z extends OpMode{

    List<LynxModule> allHubs;

    private final Pose startPose = new Pose(9,67.5, Math.toRadians(0));  // This is where the robot starts

    //Points of Interest
    Point rungPoint1 = new Point(33.5, 67.5, Point.CARTESIAN);
    Point rungPoint2 = new Point(33.5, 66, Point.CARTESIAN);
    Point rungPoint3 = new Point(33.5, 64.5, Point.CARTESIAN);
    Point rungPoint4 = new Point(33.5, 69, Point.CARTESIAN);
    Point rungPoint5 = new Point(33.5, 70.5, Point.CARTESIAN);
    Point rungPoint1a = new Point(25, 67.5, Point.CARTESIAN);
    Point rungPoint2a = new Point(25, 66, Point.CARTESIAN);
    Point rungPoint3a = new Point(25, 64.5, Point.CARTESIAN);
    Point rungPoint4a = new Point(25, 69, Point.CARTESIAN);
    Point rungPoint5a = new Point(25, 70.5, Point.CARTESIAN);

    Point rungPointControl1 = new Point(20,28, Point.CARTESIAN);
    Point rungPointControl2 = new Point(20, 66, Point.CARTESIAN);

    Point samplecurvepoint1 = new Point(19,22, Point.CARTESIAN);
    Point samplecurvepoint2 = new Point(72,48, Point.CARTESIAN);
    Point samplecurvepoint3 = new Point(72,30, Point.CARTESIAN);
    Point samplecurvepoint4 = new Point(72,20, Point.CARTESIAN);

    Point samplepoint1 =      new Point(62,26.5, Point.CARTESIAN);
    Point samplepoint2 =      new Point(62,15.5, Point.CARTESIAN);
    Point samplepoint3 =      new Point(62,10, Point.CARTESIAN);

    Point linepoint1 =        new Point(32,25, Point.CARTESIAN);
    Point linepoint2 =        new Point(32,14, Point.CARTESIAN);

    Point pickupPoint1 = new Point(18, 11.5, Point.CARTESIAN);
    Point pickupPoint2 = new Point(13.5, 28, Point.CARTESIAN);
    Point pickupPoint3 = new Point(11.5, 28, Point.CARTESIAN);

    public Path toSample1, toSample2, toSample3, toline1, toline2, toline3, score1a;

    public PathChain score1, score2, collect3, score3, moveSamples, collect4, score4, collect5, score5, score2a, score3a, score4a, score5a;

    // Other misc. stuff
    private Follower follower;

    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;

    LeftLift leftLift;
    LeftPivot leftPivot;
    ActiveIntakeMotor intake;

    RightLift rightLift;
    RightPivot rightPivot;
    ClawAndStuff grabber;

    RobotConfig config;

    private Timer pathTimer;

    int listPointer = 0;

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
        grabber = new ClawAndStuff(this, config, leftLift, leftPivot);

        leftLift.assignPivot(leftPivot);
        leftPivot.assignLift(leftLift);

        rightLift.assignPivot(rightPivot);
        rightPivot.assignLift(rightLift);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        rightLift.setTargetPosition(0);
        rightPivot.setTargetPosition(-60);
        grabber.Collect();
        leftLift.setTargetPosition(0);
        leftPivot.setTargetPosition(-55);
        grabber.closeClaw();
    }

    @Override
    public void init_loop() {
        leftLift.update();
        leftPivot.update();
        rightLift.update();
        rightPivot.update();
    }



    public void buildPaths() {
        score1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(startPose), rungPoint1a)))
                .build();

        toSample1 = new Path(new BezierCurve(rungPoint1, samplecurvepoint1, samplecurvepoint2, samplepoint1));
        toSample2 = new Path(new BezierCurve(linepoint1, samplecurvepoint3, samplepoint2));
        toSample3 = new Path(new BezierCurve(linepoint2, samplecurvepoint4, samplepoint3));

        toline1 = new Path(new BezierLine(samplepoint1, linepoint1));
        toline2 = new Path(new BezierLine(samplepoint2, linepoint2));
        toline3 = new Path(new BezierLine(samplepoint2, pickupPoint1));

        score1a = new Path(new BezierLine(rungPoint1a, rungPoint1));

        score2a = follower.pathBuilder()
                .addPath(new Path(new BezierLine(rungPoint2a, rungPoint2)))
                .addPath(new Path(new BezierCurve(rungPoint2, rungPointControl2, rungPointControl1, pickupPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(2)
                .addParametricCallback(0, () -> {
                    grabber.Collect();
                    grabber.openClaw();
                })
                .build();
        score3a = follower.pathBuilder()
                .addPath(new Path(new BezierLine(rungPoint3a, rungPoint3)))
                .addPath(new Path(new BezierCurve(rungPoint3, rungPointControl2, rungPointControl1, pickupPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(2)
                .addParametricCallback(0, () -> {
                    grabber.Collect();
                    grabber.openClaw();
                })
                .build();
        score4a = follower.pathBuilder()
                .addPath(new Path(new BezierLine(rungPoint4a, rungPoint4)))
                .addPath(new Path(new BezierCurve(rungPoint4, rungPointControl2, rungPointControl1, pickupPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(2)
                .addParametricCallback(0, () -> {
                    grabber.Collect();
                    grabber.openClaw();
                })
                .build();
        score5a = follower.pathBuilder()
                .addPath(new Path(new BezierLine(rungPoint5a, rungPoint5)))
                .addPath(new Path(new BezierLine(rungPoint5, pickupPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0, () -> {
                    grabber.Collect();
                })
                .build();

        moveSamples = follower.pathBuilder()
                .addPath(score1a)
                .addPath(toSample1)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0, () -> leftLift.setTargetPosition(0))
                .addPath(toline1)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0, () -> grabber.Collect())
                .addPath(toSample2)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(toline2)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(toSample3)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(toline3)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        score2 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(pickupPoint1, rungPointControl1, rungPointControl2, rungPoint2a)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        collect3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickupPoint2, pickupPoint3)))
                .setZeroPowerAccelerationMultiplier(2)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        score3 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(pickupPoint3, rungPointControl1, rungPointControl2, rungPoint3a)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        collect4 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickupPoint2, pickupPoint3)))
                .setZeroPowerAccelerationMultiplier(2)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        score4 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(pickupPoint3, rungPointControl1, rungPointControl2, rungPoint4a)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        collect5 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickupPoint2, pickupPoint3)))
                .setZeroPowerAccelerationMultiplier(2)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        score5 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(pickupPoint3, rungPointControl1, rungPointControl2, rungPoint5a)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void routine() {
        switch (listPointer) {
            case 0:
                follower.followPath(score1);
                listPointer = 1;
                break;
            case 1:
                if (!follower.isBusy() && leftPivot.getPosition() >= 34 && leftLift.getPosition() >= 11) {
                    follower.followPath(moveSamples);
                    listPointer = 2;
                }
            case 2:
                if (!follower.isBusy()) {
                    listPointer = 3;
                    pathTimer.resetTimer();
                    grabber.closeClaw();
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    grabber.Score();
                    if (leftPivot.getPosition() >= 0) {
                        follower.followPath(score2);
                        listPointer = 4;
                    }
                }
                break;
            case 4:
                if (!follower.isBusy() && leftPivot.getPosition() >= 34 && leftLift.getPosition() >= 11) {
                    follower.followPath(score2a);
                    listPointer = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    grabber.openClaw();
                    follower.followPath(collect3);
                    listPointer = 6;
                }
            case 6:
                if (!follower.isBusy()) {
                    listPointer = 7;
                    pathTimer.resetTimer();
                    grabber.closeClaw();
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    grabber.Score();
                    if (leftPivot.getPosition() >= 0) {
                        follower.followPath(score3);
                        listPointer = 8;
                    }
                }
                break;
            case 8:
                if (!follower.isBusy() && leftPivot.getPosition() >= 34 && leftLift.getPosition() >= 11) {
                    follower.followPath(score3a);
                    listPointer = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    grabber.openClaw();
                    follower.followPath(collect4);
                    listPointer = 10;
                }
            case 10:
                if (!follower.isBusy()) {
                    listPointer = 11;
                    pathTimer.resetTimer();
                    grabber.closeClaw();
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    grabber.Score();
                    if (leftPivot.getPosition() >= 0) {
                        follower.followPath(score4);
                        listPointer = 12;
                    }
                }
                break;
            case 12:
                if (!follower.isBusy() && leftPivot.getPosition() >= 34 && leftLift.getPosition() >= 11) {
                    follower.followPath(score4a);
                    listPointer = 13;
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    grabber.openClaw();
                    follower.followPath(collect5);
                    listPointer = 14;
                }
            case 14:
                if (!follower.isBusy()) {
                    listPointer = 15;
                    pathTimer.resetTimer();
                    grabber.closeClaw();
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    listPointer = 16;
                    pathTimer.resetTimer();
                    grabber.closeClaw();
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    grabber.Score();
                    if (leftPivot.getPosition() >= 0) {
                        follower.followPath(score5);
                        listPointer = 17;
                    }
                }
                break;
            case 17:
                if (!follower.isBusy() && leftPivot.getPosition() >= 34 && leftLift.getPosition() >= 11) {
                    follower.followPath(score5a);
                    listPointer = 18;
                }
                break;
        }
    }

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        deltaTime = frameTimer.seconds();
        frameTimer.reset();

        routine();
        follower.update();
        leftLift.update();
        leftPivot.update();
        rightLift.update();
        rightPivot.update();
        intake.update();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("deltaTime", deltaTime);
        telemetry.addData("waitTime", time);
        telemetry.addData("pathtimer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Path State", listPointer);
        telemetry.update();
    }

    @Override
    public void start() {
        buildPaths();
        grabber.Score();
        pathTimer.resetTimer();

        deltaTime = 0;
        frameTimer.reset();
        // Put first path instead of brackets
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }
    /**
     * 8==D
     */
}

