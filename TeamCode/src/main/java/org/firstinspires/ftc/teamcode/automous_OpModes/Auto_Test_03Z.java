package org.firstinspires.ftc.teamcode.automous_OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Lift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Pivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveIntake;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ReadOnlyRuntime;

import java.util.List;

@Autonomous(name = "SoupcOpMode_0-3-Z 1.0.4")
public class Auto_Test_03Z extends OpMode{


    enum State {
        TO_RUNG1,
        TO_RUNG2,
        TO_RUNG3,
        TO_PICKUP2,
        TO_OBSERVE,
        MOVE_SAMPLE,
        IDLE
    }

    State currentState = State.IDLE;

    List<LynxModule> allHubs;

    private final Pose startPose = new Pose(9,66, Math.toRadians(0));  // This is where the robot starts

    //Points of Interest
    private Point rungpoint1 =        new Point(36,66, Point.CARTESIAN);
    private Point rungpoint2 =        new Point(36.25,71, Point.CARTESIAN);
    private Point rungpoint3 =        new Point(36.25,75, Point.CARTESIAN);

    private Point samplepoint =       new Point(60,25, Point.CARTESIAN);

    private Point linepoint =         new Point(32,25, Point.CARTESIAN);

    private Point pickuppoint =       new Point(10,43, Point.CARTESIAN); //TODO: Make this more accurate

    private Point observepoint =      new Point(10,10, Point.CARTESIAN);


    private Point curvepoint =        new Point(28,69, Point.CARTESIAN);

    private Point samplecurvepoint1 = new Point(19,22, Point.CARTESIAN);
    private Point samplecurvepoint2 = new Point(72,48, Point.CARTESIAN);

    private Point pickupcurvepoint1 =  new Point(40,60, Point.CARTESIAN);
    private Point pickupcurvepoint2 =  new Point(24,72, Point.CARTESIAN);



    public Path toRung1, toRung2, toRung3, toSample, toline, toPickup1, toPickup2, toObserve;
    public PathChain moveSample;

    // Other misc. stuff
    private Follower follower;

    private final ReadOnlyRuntime runtime = new ReadOnlyRuntime();
    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;

    Lift lift;
    Pivot pivot;
    ActiveIntake intake;
    RobotConfig config;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk


        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        config = new RobotConfig(this);

        lift = new Lift(ControlAxis.ControlMode.positionControl,this, config);
        pivot = new Pivot(ControlAxis.ControlMode.positionControl,this, config);
        intake = new ActiveIntake(this, config);

        lift.assignPivot(pivot);
        pivot.assignLift(lift);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
    }



    public void buildPaths() {


        toRung1 = new Path(new BezierLine(new Point(startPose), rungpoint1));
        toRung2 = new Path(new BezierCurve(pickuppoint, curvepoint, rungpoint2));
        toRung3 = new Path(new BezierCurve(pickuppoint, curvepoint, rungpoint3));

        toPickup1 = new Path(new BezierCurve(linepoint, pickupcurvepoint1, pickupcurvepoint2, pickuppoint));
        toPickup2 = new Path(new BezierCurve(rungpoint2, curvepoint, pickuppoint));

        toSample = new Path(new BezierCurve(rungpoint1, samplecurvepoint1, samplecurvepoint2, samplepoint));

        toline = new Path(new BezierLine(samplepoint, linepoint));

        toObserve = new Path(new BezierCurve(rungpoint3, curvepoint, observepoint));


        toRung1.setConstantHeadingInterpolation(Math.toRadians(0));
        toRung2.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0));
        toRung3.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0));

        toPickup2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270));

        toObserve.setConstantHeadingInterpolation(Math.toRadians(0));


        moveSample = follower.pathBuilder()
                .addPath(toSample)
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .addPath(toline)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(toPickup1)
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();
    }

    public void autonomousPathUpdate() {

        switch (currentState) {
            case TO_RUNG1:
                lift.setTargetPosition(11.5);
                pivot.setTargetPosition(15);
                if (follower.atParametricEnd()) {
                    telemetry.addLine("im here");
                    lift.setTargetPosition(0);
                    if (lift.getPosition() <= 1) {
                        telemetry.addLine("im here 2");
                        currentState = State.MOVE_SAMPLE;
                        follower.followPath(moveSample);
                    }
                }
                break;

            case MOVE_SAMPLE:

                if (follower.atParametricEnd()) {
                    pivot.setTargetPosition(90);
                    if (pivot.getPosition() >= 87.5) {
                        intake.intakeForDuration(0.5);
                        currentState = State.TO_RUNG2;
                        follower.followPath(toRung2);
                    }
                }
                break;

            case TO_RUNG2:
                pivot.setTargetPosition(13);
                lift.setTargetPosition(11.5);
                if (follower.atParametricEnd()) {
                    lift.setTargetPosition(0);
                    if (lift.getPosition() <= 1) {
                        currentState = State.TO_PICKUP2;
                        follower.followPath(toPickup2);
                    }
                }
                break;

            case TO_PICKUP2:

                if (follower.atParametricEnd()) {
                    pivot.setTargetPosition(90);
                    if (pivot.getPosition() >= 87.5) {
                        intake.intakeForDuration(0.5);
                        currentState = State.TO_RUNG3;
                        follower.followPath(toRung3);
                    }
                }
                break;

            case TO_RUNG3:
                pivot.setTargetPosition(13);
                lift.setTargetPosition(11.5);
                if (follower.atParametricEnd()) {
                    lift.setTargetPosition(0);
                    if (lift.getPosition() <= 1) {
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
        telemetry.addData("runTime", runtime);
        telemetry.addData("waitTime", time);
        telemetry.update();
    }



    @Override
    public void start() {
        buildPaths();
        deltaTime = 0;
        frameTimer.reset();
        // Put first path instead of brackets
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