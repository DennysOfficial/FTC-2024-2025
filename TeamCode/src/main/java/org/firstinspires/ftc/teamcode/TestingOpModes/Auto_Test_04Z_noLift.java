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
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ReadOnlyRuntime;

import java.util.List;

@Disabled
@Autonomous(name = "SoupcOpMode_0-4-Z 0.0.0 Test")
public class Auto_Test_04Z_noLift extends OpMode{


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

    private final Pose startPose = new Pose(9,66, Math.toRadians(0));  // This is where the robot starts

    //Points of Interest
    private Point rungpoint1 =        new Point(38,66, Point.CARTESIAN);
    private Point rungpoint2 =        new Point(38,69, Point.CARTESIAN);
    private Point rungpoint3 =        new Point(38,72, Point.CARTESIAN);
    private Point rungpoint4 =        new Point(38,75, Point.CARTESIAN);

    private Point samplepoint1 =       new Point(60,24, Point.CARTESIAN);
    private Point samplepoint2 =       new Point(60,13, Point.CARTESIAN);

    private Point linepoint1 =         new Point(18,24, Point.CARTESIAN);
    private Point linepoint2 =         new Point(18,13, Point.CARTESIAN);

    private Point pickuppoint =       new Point(10,43, Point.CARTESIAN); //TODO: Make this more accurate

    private Point observepoint =      new Point(10,10, Point.CARTESIAN);


    private Point curvepoint =        new Point(28,69, Point.CARTESIAN);

    private Point samplecurvepoint1 = new Point(19,22, Point.CARTESIAN);
    private Point samplecurvepoint2 = new Point(72,48, Point.CARTESIAN);
    private Point samplecurvepoint3 = new Point(72,30, Point.CARTESIAN);

    private Point pickupcurvepoint =  new Point(48,48, Point.CARTESIAN);



    public Path toRung1, toRung2, toRung3, toRung4, toSample1, toSample2, toline1, toline2, toPickup1, toPickup2, toPickup3, toObserve;
    public PathChain moveSamples;

    // Other misc. stuff
    private Follower follower;

    private final ReadOnlyRuntime runtime = new ReadOnlyRuntime();
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
        toRung4 = new Path(new BezierCurve(pickuppoint, curvepoint, rungpoint4));

        toPickup1 = new Path(new BezierCurve(linepoint2, pickupcurvepoint, pickuppoint));
        toPickup2 = new Path(new BezierCurve(rungpoint2, curvepoint, pickuppoint));
        toPickup3 = new Path(new BezierCurve(rungpoint3, curvepoint, pickuppoint));

        toSample1 = new Path(new BezierCurve(rungpoint1, samplecurvepoint1, samplecurvepoint2, samplepoint1));
        toSample2 = new Path(new BezierCurve(linepoint1, samplecurvepoint3, samplepoint2));

        toline1 = new Path(new BezierLine(samplepoint1, linepoint1));
        toline1 = new Path(new BezierLine(samplepoint2, linepoint2));

        toObserve = new Path(new BezierCurve(rungpoint4, curvepoint, observepoint));


        toRung1.setConstantHeadingInterpolation(Math.toRadians(0));
        toRung2.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0));
        toRung3.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0));
        toRung4.setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0));

        toPickup2.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270));
        toPickup3.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270));

        toObserve.setConstantHeadingInterpolation(Math.toRadians(0));


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

                if (!follower.isBusy()) {

                        currentState = State.MOVE_SAMPLES;
                        follower.followPath(moveSamples);

                }
                break;

            case MOVE_SAMPLES:

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

                        currentState = State.TO_PICKUP3;
                        follower.followPath(toPickup3);

                }
                break;

            case TO_PICKUP3:

                if (!follower.isBusy()) {

                        currentState = State.TO_RUNG4;
                        follower.followPath(toRung4);

                }
                break;

            case TO_RUNG4:

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



        follower.update();

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