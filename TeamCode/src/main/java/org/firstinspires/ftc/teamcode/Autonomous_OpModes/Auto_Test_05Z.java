package org.firstinspires.ftc.teamcode.Autonomous_OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.SpecimenArm;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.RightLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.RightPivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveSpecimenClaw;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import java.util.List;

@Autonomous(name = "Superman Test: SoupcOpMode_0-5-Z 0.0.1", group = "SoupcOpModes")
public class Auto_Test_05Z extends OpMode{

    List<LynxModule> allHubs;

    private final Pose startPose = new Pose(9.25,48.5, Math.toRadians(0));  // This is where the robot starts

    //Points of Interest
    Point rungPoint1 = new Point(38.5, 67.5, Point.CARTESIAN);
    Point rungPoint2 = new Point(32, 66, Point.CARTESIAN);
    Point rungPoint3 = new Point(32, 64.5, Point.CARTESIAN);
    Point rungPoint4 = new Point(32, 69, Point.CARTESIAN);
    Point rungPoint5 = new Point(32, 70.5, Point.CARTESIAN);
    Point rungPoint1a = new Point(17, 67.5, Point.CARTESIAN);
    Point rungPoint2a = new Point(17, 66, Point.CARTESIAN);
    Point rungPoint3a = new Point(17, 64.5, Point.CARTESIAN);
    Point rungPoint4a = new Point(17, 69, Point.CARTESIAN);
    Point rungPoint5a = new Point(17, 70.5, Point.CARTESIAN);

    Point rungPointControl1 = new Point(20,28, Point.CARTESIAN);
    Point rungPointControl2 = new Point(20, 66, Point.CARTESIAN);

    Point samplecurvepoint1 = new Point(19,22, Point.CARTESIAN);
    Point samplecurvepoint2 = new Point(72,48, Point.CARTESIAN);
    Point samplecurvepoint3 = new Point(72,30, Point.CARTESIAN);
    Point samplecurvepoint4 = new Point(72,20, Point.CARTESIAN);

    Point samplepoint1 =      new Point(62,26.5, Point.CARTESIAN);
    Point samplepoint2 =      new Point(62,17, Point.CARTESIAN);
    Point samplepoint3 =      new Point(62,11.25, Point.CARTESIAN);

    Point linepoint1 =        new Point(32,25, Point.CARTESIAN);
    Point linepoint2 =        new Point(32,17, Point.CARTESIAN);

    Point pickupPoint1 = new Point(14.5, 11.25, Point.CARTESIAN);
    Point pickupPoint2 = new Point(14.5, 26, Point.CARTESIAN);
    Point pickupPoint3 = new Point(12, 26, Point.CARTESIAN);

    public Path toSample1, toSample2, toSample3, toline1, toline2, toline3, score1a;

    public PathChain score1, score2, collect3, score3, moveSamples, collect4, score4, collect5, score5, score2a, score3a, score4a, score5a;

    // Other misc. stuff
    private Follower follower;

    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;

    RobotConfig config;

    private Timer pathTimer;

    int listPointer = 0;

    RightLift rightLift;
    RightPivot rightPivot;
    ActiveSpecimenClaw grabber;

    SpecimenArm spArm;
    Servo wristServo;

    @Override
    public void init() {

        pathTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk


        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        config = new RobotConfig(this);

        rightLift = new RightLift(ControlAxis.ControlMode.positionControl,this, config);
        rightPivot = new RightPivot(ControlAxis.ControlMode.positionControl,this, config);
        grabber = new ActiveSpecimenClaw(this, config);

        spArm = new SpecimenArm(this, config);

        rightLift.assignPivot(rightPivot);
        rightPivot.assignLift(rightLift);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        rightLift.setTargetPosition(0);
        rightPivot.setTargetPosition(-50);
        spArm.armState = SpecimenArm.SpecimenArmState.rest;
        grabber.closeClawHard();

        wristServo = this.hardwareMap.get(Servo.class, config.deviceConfig.sampleClawDepositWristServo);
    }

    @Override
    public void init_loop() {
        rightLift.update();
        rightPivot.update();
        spArm.autoUpdate();
    }



    public void buildPaths() {
        score1 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(startPose), rungPointControl2, rungPoint1a)))
                .setPathEndTimeoutConstraint(100)
                .setPathEndVelocityConstraint(0.15)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        toSample1 = new Path(new BezierCurve(rungPoint1, samplecurvepoint1, samplecurvepoint2, samplepoint1));
        toSample2 = new Path(new BezierCurve(linepoint1, samplecurvepoint3, samplepoint2));
        toSample3 = new Path(new BezierCurve(linepoint2, samplecurvepoint4, samplepoint3));

        toline1 = new Path(new BezierLine(samplepoint1, linepoint1));
        toline2 = new Path(new BezierLine(samplepoint2, linepoint2));
        toline3 = new Path(new BezierLine(samplepoint3, pickupPoint1));

        score1a = new Path(new BezierLine(rungPoint1a, rungPoint1));
        score1a.setPathEndTimeoutConstraint(200);
        score1a.setPathEndVelocityConstraint(0.15);

        score2a = follower.pathBuilder()
                .addPath(new Path(new BezierLine(rungPoint2a, rungPoint2)))
                .addPath(new Path(new BezierCurve(rungPoint2, rungPointControl2, rungPointControl1, pickupPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(100)
                .setPathEndVelocityConstraint(0.15)
                .addParametricCallback(0, () -> {
                    spArm.armState = SpecimenArm.SpecimenArmState.collect;
                    grabber.openClaw();
                })
                .build();
        score3a = follower.pathBuilder()
                .addPath(new Path(new BezierLine(rungPoint3a, rungPoint3)))
                .addPath(new Path(new BezierCurve(rungPoint3, rungPointControl2, rungPointControl1, pickupPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(100)
                .setPathEndVelocityConstraint(0.15)
                .addParametricCallback(0, () -> {
                    spArm.armState = SpecimenArm.SpecimenArmState.collect;
                    grabber.openClaw();
                })
                .build();
        score4a = follower.pathBuilder()
                .addPath(new Path(new BezierLine(rungPoint4a, rungPoint4)))
                .addPath(new Path(new BezierCurve(rungPoint4, rungPointControl2, rungPointControl1, pickupPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(100)
                .setPathEndVelocityConstraint(0.15)
                .addParametricCallback(0, () -> {
                    spArm.armState = SpecimenArm.SpecimenArmState.collect;
                    grabber.openClaw();
                })
                .build();
        score5a = follower.pathBuilder()
                .addPath(new Path(new BezierLine(rungPoint5a, rungPoint5)))
                .addParametricCallback(1, () -> {
                    spArm.armState = SpecimenArm.SpecimenArmState.collect;
                    grabber.openClaw();
                })
                .build();

        moveSamples = follower.pathBuilder()
                .addPath(toSample1)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0, () -> {
                    spArm.armState = SpecimenArm.SpecimenArmState.collect;
                    grabber.openClaw();
                })
                .addPath(toline1)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(toSample2)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(toline2)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(toSample3)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(toline3)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(5)
                .build();

        score2 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(pickupPoint1, rungPointControl1, rungPointControl2, rungPoint2a)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(50)
                .setPathEndVelocityConstraint(0.3)
                .build();

        collect3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickupPoint2, pickupPoint3)))
                .setZeroPowerAccelerationMultiplier(5)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(350)
                .build();
        score3 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(pickupPoint3, rungPointControl1, rungPointControl2, rungPoint3a)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(50)
                .setPathEndVelocityConstraint(0.3)
                .build();

        collect4 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickupPoint2, pickupPoint3)))
                .setZeroPowerAccelerationMultiplier(5)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(350)
                .build();

        score4 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(pickupPoint3, rungPointControl1, rungPointControl2, rungPoint4a)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(50)
                .setPathEndVelocityConstraint(0.3)
                .build();

        collect5 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickupPoint2, pickupPoint3)))
                .setZeroPowerAccelerationMultiplier(5)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(350)
                .build();

        score5 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(pickupPoint3, rungPointControl1, rungPointControl2, rungPoint5a)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(50)
                .setPathEndVelocityConstraint(0.3)
                .build();
    }

    public void routine() {
        switch (listPointer) {
            case 0:
                if (!follower.isBusy() && !spArm.pivotIsBusy()) {
                    follower.followPath(score1a);
                    if (follower.isBusy()) listPointer = 1;
                }
                break;
            case 1:
                if (!follower.isBusy() && !spArm.pivotIsBusy()) {
                    follower.followPath(moveSamples);
                    if (follower.isBusy()) listPointer = 2;
                    pathTimer.resetTimer();
                }
                break;
            case 2:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= 12) {
                    listPointer = 3;
                    pathTimer.resetTimer();
                    grabber.closeClawHard();
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() >= 0.35) {
                    spArm.armState = SpecimenArm.SpecimenArmState.movingToScore;
                    if (spArm.liftPos() >= 7) {
                        follower.followPath(score2);
                        if (follower.isBusy()) listPointer = 4;
                    }
                }
                break;
            case 4:
                if (!follower.isBusy() && !spArm.pivotIsBusy()) {
                    follower.followPath(score2a);
                    if (follower.isBusy()) listPointer = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    grabber.openClaw();
                    follower.followPath(collect3);
                    if (follower.isBusy()) listPointer = 6;
                    pathTimer.resetTimer();
                }
            case 6:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= 7) {
                    listPointer = 7;
                    pathTimer.resetTimer();
                    grabber.closeClawHard();
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() >= 0.35) {
                    spArm.armState = SpecimenArm.SpecimenArmState.movingToScore;
                    if (spArm.liftPos() >= 7) {
                        follower.followPath(score3);
                        if (follower.isBusy()) listPointer = 8;
                    }
                }
                break;
            case 8:
                if (!follower.isBusy() && !spArm.pivotIsBusy()) {
                    follower.followPath(score3a);
                    if (follower.isBusy()) listPointer = 9;
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    grabber.openClaw();
                    follower.followPath(collect4);
                    if (follower.isBusy()) listPointer = 10;
                    pathTimer.resetTimer();
                }
            case 10:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= 7) {
                    listPointer = 11;
                    pathTimer.resetTimer();
                    grabber.closeClawHard();
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() >= 0.35) {
                    spArm.armState = SpecimenArm.SpecimenArmState.movingToScore;
                    if (spArm.liftPos() >= 7) {
                        follower.followPath(score4);
                        if (follower.isBusy()) listPointer = 12;
                    }
                }
                break;
            case 12:
                if (!follower.isBusy() && !spArm.pivotIsBusy()) {
                    follower.followPath(score4a);
                    if (follower.isBusy()) listPointer = 13;
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    grabber.openClaw();
                    follower.followPath(collect5);
                    if (follower.isBusy()) listPointer = 14;
                }
            case 14:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() >= 7) {
                    listPointer = 16;
                    pathTimer.resetTimer();
                    grabber.closeClawHard();
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() >= 0.35) {
                    spArm.armState = SpecimenArm.SpecimenArmState.movingToScore;
                    if (spArm.liftPos() >= 7) {
                        follower.followPath(score5);
                        if (follower.isBusy()) listPointer = 17;
                    }
                }
                break;
            case 17:
                if (!follower.isBusy() && !spArm.pivotIsBusy()) {
                    follower.followPath(score5a);
                    if (follower.isBusy()) listPointer = 69;
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
        rightLift.update();
        rightPivot.update();
        spArm.autoUpdate();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("deltaTime", deltaTime);
        telemetry.addData("waitTime", time);
        telemetry.addData("pathtimer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Path State", listPointer);
        telemetry.addData("path:", follower.getCurrentPath().toString());
        telemetry.addData("busy", follower.isBusy());
        telemetry.update();
    }

    @Override
    public void start() {
        buildPaths();
        pathTimer.resetTimer();
        spArm.armState = SpecimenArm.SpecimenArmState.movingToScore;

        deltaTime = 0;
        frameTimer.reset();
        // Put first path instead of brackets
        follower.followPath(score1);
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }
    /**
     * 8==D
     */
}

