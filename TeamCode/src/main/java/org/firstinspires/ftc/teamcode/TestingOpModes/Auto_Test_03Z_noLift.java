package org.firstinspires.ftc.teamcode.TestingOpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.List;

@Disabled
@Autonomous(name = "SoupcOpMode_0-3-Z 0.0.0 Test")
public class Auto_Test_03Z_noLift extends OpMode{


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
    private Point rungpoint1 =        new Point(38,66, Point.CARTESIAN);
    private Point rungpoint2 =        new Point(38,69, Point.CARTESIAN);
    private Point rungpoint3 =        new Point(38,72, Point.CARTESIAN);

    private Point samplepoint =       new Point(60,25, Point.CARTESIAN);

    private Point linepoint =         new Point(22,25, Point.CARTESIAN);

    private Point pickuppoint =       new Point(10,43, Point.CARTESIAN); //TODO: Make this more accurate

    private Point observepoint =      new Point(10,10, Point.CARTESIAN);


    private Point curvepoint =        new Point(28,69, Point.CARTESIAN);

    private Point samplecurvepoint1 = new Point(19,22, Point.CARTESIAN);
    private Point samplecurvepoint2 = new Point(72,48, Point.CARTESIAN);

    private Point pickupcurvepoint =  new Point(48,48, Point.CARTESIAN);



    public Path toRung1, toRung2, toRung3, toSample, toline, toPickup1, toPickup2, toObserve;
    public PathChain moveSample;

    // Other misc. stuff
    private Follower follower;

    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;


    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk


        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }



        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
    }



    public void buildPaths() {


        toRung1 = new Path(new BezierLine(new Point(startPose), rungpoint1));
        toRung2 = new Path(new BezierCurve(pickuppoint, curvepoint, rungpoint2));
        toRung3 = new Path(new BezierCurve(pickuppoint, curvepoint, rungpoint3));

        toPickup1 = new Path(new BezierCurve(linepoint, pickupcurvepoint, pickuppoint));
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

                if (!follower.isBusy()) {

                        currentState = State.MOVE_SAMPLE;
                        follower.followPath(moveSample);

                }
                break;

            case MOVE_SAMPLE:

                if (!follower.isBusy()) {


                        currentState = State.TO_RUNG2;
                        follower.followPath(toRung2);

                }
                break;

            case TO_RUNG2:

                if (!follower.isBusy()) {

                        currentState = State.TO_PICKUP2;
                        follower.followPath(toPickup2);

                }
                break;

            case TO_PICKUP2:

                if (!follower.isBusy()) {


                        currentState = State.TO_RUNG3;
                        follower.followPath(toRung3);

                }
                break;

            case TO_RUNG3:

                if (!follower.isBusy()) {

                        currentState = State.TO_OBSERVE;
                        follower.followPath(toObserve);

                }
                break;

            case TO_OBSERVE:

                if (!follower.isBusy()) {
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

        autonomousPathUpdate();

        follower.update();

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