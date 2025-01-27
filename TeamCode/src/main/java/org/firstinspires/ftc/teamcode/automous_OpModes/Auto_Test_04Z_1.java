package org.firstinspires.ftc.teamcode.automous_OpModes;


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
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveIntakeServo;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.List;

@Autonomous(name = "SoupcOpMode_0-4-Z 1.2.9")
public class Auto_Test_04Z_1 extends OpMode{


    enum State {
        TO_RUNG1,
        TO_RUNG2,
        TO_RUNG3,
        TO_RUNG4,
        TO_PICKUP2,
        TO_PICKUP3,
        TO_OBSERVE,
        MOVE_SAMPLES,
        IDLE
    }

    State currentState = State.IDLE;

    List<LynxModule> allHubs;

    private final Pose startPose = new Pose(9,64.5, Math.toRadians(0));  // This is where the robot starts

    //Points of Interest
    private Point rungpoint1 =        new Point(32.75,64.5, Point.CARTESIAN);
    private Point rungpoint2 =        new Point(32.75,70, Point.CARTESIAN);
    private Point rungpoint3 =        new Point(33,76, Point.CARTESIAN);
    private Point rungpoint4 =        new Point(33,82, Point.CARTESIAN);

    private Point samplepoint1 =      new Point(62,25, Point.CARTESIAN);
    private Point samplepoint2 =      new Point(62,15.5, Point.CARTESIAN);

    private Point linepoint1 =        new Point(32,25, Point.CARTESIAN);
    private Point linepoint2 =        new Point(32,15.5, Point.CARTESIAN);

    private Point pickuppoint =       new Point(15,38, Point.CARTESIAN); //TODO: Make this more accurate

    private Point observepoint =      new Point(10,10, Point.CARTESIAN);


    private Point curvepoint =        new Point(15,60, Point.CARTESIAN);

    private Point samplecurvepoint1 = new Point(19,22, Point.CARTESIAN);
    private Point samplecurvepoint2 = new Point(72,48, Point.CARTESIAN);
    private Point samplecurvepoint3 = new Point(72,30, Point.CARTESIAN);

    private Point pickupcurvepoint1 =  new Point(48,48, Point.CARTESIAN);
    private Point pickupcurvepoint2 =  new Point(24,60, Point.CARTESIAN);



    public Path toRung1, toRung2, toRung3, toRung4, toSample1, toSample2, toline1, toline2, toPickup1, toPickup2, toPickup3, toObserve;
    public PathChain moveSamples;

    // Other misc. stuff
    private Follower follower;

    double liftPosRung = 14;
    double pivotPosRung = 20.25;

    double pivotPosObs = 90;

    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;

    LeftLift lift;
    LeftPivot pivot;
    ActiveIntakeServo intake;
    RobotConfig config;

    private Timer pathTimer;

    @Override
    public void init() {

        pathTimer = new Timer();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk


        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        config = new RobotConfig(this);

        lift = new LeftLift(ControlAxis.ControlMode.positionControl,this, config);
        pivot = new LeftPivot(ControlAxis.ControlMode.positionControl,this, config);
        intake = new ActiveIntakeServo(this, config);

        lift.assignPivot(pivot);
        pivot.assignLift(lift);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
    }



    public void buildPaths() {


        toRung1 = new Path(new BezierLine(new Point(startPose), rungpoint1));
        toRung2 = new Path(new BezierCurve(pickuppoint, curvepoint, rungpoint2));
        toRung3 = new Path(new BezierCurve(pickuppoint, curvepoint, rungpoint3));
        toRung4 = new Path(new BezierCurve(pickuppoint, curvepoint, rungpoint4));

        toPickup1 = new Path(new BezierCurve(linepoint2, pickupcurvepoint1, pickupcurvepoint2, pickuppoint));
        toPickup2 = new Path(new BezierCurve(rungpoint2, curvepoint, pickuppoint));
        toPickup3 = new Path(new BezierCurve(rungpoint3, curvepoint, pickuppoint));

        toSample1 = new Path(new BezierCurve(rungpoint1, samplecurvepoint1, samplecurvepoint2, samplepoint1));
        toSample2 = new Path(new BezierCurve(linepoint1, samplecurvepoint3, samplepoint2));

        toline1 = new Path(new BezierLine(samplepoint1, linepoint1));
        toline2 = new Path(new BezierLine(samplepoint2, linepoint2));

        toObserve = new Path(new BezierCurve(rungpoint4, curvepoint, observepoint));


        toRung1.setConstantHeadingInterpolation(Math.toRadians(0));
        toRung2.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0));
        toRung3.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0));
        toRung4.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0));

        toPickup2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270));
        toPickup3.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270));

        toObserve.setConstantHeadingInterpolation(Math.toRadians(0));

        toPickup2.setZeroPowerAccelerationMultiplier(4);
        toPickup3.setZeroPowerAccelerationMultiplier(4);

        toPickup2.setPathEndTranslationalConstraint(0.075);
        toPickup3.setPathEndTranslationalConstraint(0.075);

        toPickup2.setPathEndHeadingConstraint(0.006);
        toPickup3.setPathEndHeadingConstraint(0.006);

        toPickup2.setPathEndTimeoutConstraint(750);
        toPickup3.setPathEndTimeoutConstraint(750);


        moveSamples = follower.pathBuilder()
                .addPath(toSample1)
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .addPath(toline1)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(toSample2)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(toline2)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(toPickup1)
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();
    }

    public void autonomousPathUpdate() {

        switch (currentState) {
            case TO_RUNG1:
                lift.setTargetPosition(liftPosRung);
                if (lift.getPosition() >= 3){
                    pivot.setTargetPosition(pivotPosRung);
                }
                if (follower.atParametricEnd() || pathTimer.getElapsedTimeSeconds() >= 1.75) {
                    telemetry.addLine("im here");
                    lift.setTargetPosition(0);
                    if (lift.getPosition() <= 1) {
                        pathTimer.resetTimer();
                        telemetry.addLine("im here 2");
                        intake.outtakeForDuration(0.75);
                        currentState = State.MOVE_SAMPLES;
                        follower.followPath(moveSamples);
                    }
                }
                break;

            case MOVE_SAMPLES:

                if (follower.atParametricEnd()) {
                    pivot.setTargetPosition(pivotPosObs);
                    if (pivot.getPosition() >= 87 || pathTimer.getElapsedTimeSeconds() >= 12) {
                        intake.intakeForDuration(0.5);
                        pathTimer.resetTimer();
                        currentState = State.TO_RUNG2;
                        follower.followPath(toRung2);
                    }
                }
                break;
            case TO_RUNG2:
                lift.setTargetPosition(liftPosRung);
                if (lift.getPosition() >= 3){
                    pivot.setTargetPosition(pivotPosRung);
                }
                if (follower.atParametricEnd() || pathTimer.getElapsedTimeSeconds() >= 3) {
                    lift.setTargetPosition(0);
                    if (lift.getPosition() <= 1) {
                        intake.outtakeForDuration(0.75);
                        pathTimer.resetTimer();
                        currentState = State.TO_PICKUP2;
                        follower.followPath(toPickup2);
                    }
                }
                break;

            case TO_PICKUP2:

                if (follower.atParametricEnd()) {
                    pivot.setTargetPosition(pivotPosObs);
                    if (pivot.getPosition() >= 87 || pathTimer.getElapsedTimeSeconds() >= 3) {
                        intake.intakeForDuration(0.5);
                        pathTimer.resetTimer();
                        currentState = State.TO_RUNG3;
                        follower.followPath(toRung3);
                    }
                }
                break;

            case TO_RUNG3:
                lift.setTargetPosition(liftPosRung);
                if (lift.getPosition() >= 3){
                    pivot.setTargetPosition(pivotPosRung);
                }
                if (follower.atParametricEnd() || pathTimer.getElapsedTimeSeconds() >= 3) {
                    lift.setTargetPosition(0);
                    if (lift.getPosition() <= 1) {
                        intake.outtakeForDuration(0.75);
                        pathTimer.resetTimer();
                        currentState = State.TO_PICKUP3;
                        follower.followPath(toPickup3);
                    }
                }
                break;

            case TO_PICKUP3:

                if (follower.atParametricEnd()) {
                    pivot.setTargetPosition(pivotPosObs);
                    if (pivot.getPosition() >= 87 || pathTimer.getElapsedTimeSeconds() >= 3) {
                        intake.intakeForDuration(0.5);
                        pathTimer.resetTimer();
                        currentState = State.TO_RUNG4;
                        follower.followPath(toRung4);
                    }
                }
                break;

            case TO_RUNG4:
                lift.setTargetPosition(liftPosRung);
                if (lift.getPosition() >= 3){
                    pivot.setTargetPosition(pivotPosRung);
                }
                if (follower.atParametricEnd() || pathTimer.getElapsedTimeSeconds() >= 3) {
                    lift.setTargetPosition(0);
                    if (lift.getPosition() <= 1) {
                        pathTimer.resetTimer();
                        currentState = State.TO_OBSERVE;
                        follower.followPath(toObserve);
                    }
                }
                break;

            case TO_OBSERVE:

                if (follower.atParametricEnd()) {
                    currentState = State.IDLE;
                }

            case IDLE:
                // This concludes the autonomous program
                // Team 14988 couldn't stand bees
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

        intake.closeFlap();

        follower.update();
        lift.update();
        pivot.update();
        intake.update();

        autonomousPathUpdate();

        telemetry.addData("path state", currentState);
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
        deltaTime = 0;
        frameTimer.reset();
        // Put first path instead of brackets
        pathTimer.resetTimer();
        currentState = State.TO_RUNG1;
        follower.followPath(toRung1);
    }

    @Override
    public void stop() {
    }
}

/**
 * 8==D
 */