package org.firstinspires.ftc.teamcode.automous_OpModes;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ReadOnlyRuntime;

import java.util.List;

@Autonomous(name = "SoupcOpMode_0-2-Z 1.0.2 Test")
public class Auto_Test_02Z_noLift extends OpMode {

    enum State {
        TO_RUNG_START,
        TO_PICKUP,
        TO_RUNG_2,
        TO_OBSERVE,
        LIFT1,
        LIFT2,
        INTAKE1,
        IDLE            // Our bot will enter the IDLE state when done
    }

    State currentState = State.IDLE;

    List<LynxModule> allHubs;

    private final Pose startPose = new Pose(9, 72, Math.toRadians(0));  // This is where the robot starts

    //Points of Interest
    private final Point rungpoint = new Point(38, 72, Point.CARTESIAN);
    private final Point rungpoint1 = new Point(38, 69, Point.CARTESIAN);
    private final Point curvepoint = new Point(15, 72, Point.CARTESIAN);
    private final Point observepoint = new Point(10, 10, Point.CARTESIAN);
    private final Point pickuppoint = new Point(15.5, 43, Point.CARTESIAN); // TODO: Make this more specific


    // List of paths the robot takes
    private PathChain toRungStart, toPickup, toRung2, toObserve;

    // Other misc. stuff
    private Follower follower;

    private final ReadOnlyRuntime runtime = new ReadOnlyRuntime();
    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;

    @Override
    public void init() {

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
    }


    public void buildPaths() {

        toRungStart = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), rungpoint))
                .build();


        toPickup = follower.pathBuilder()
                .addPath(new BezierCurve(rungpoint, curvepoint, pickuppoint))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                .build();


        toRung2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickuppoint, curvepoint, rungpoint1))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                .build();


        toObserve = follower.pathBuilder()
                .addPath(new BezierCurve(rungpoint1, curvepoint, observepoint))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {

        switch (currentState) {

            case TO_RUNG_START:

                if (!follower.isBusy()) {
                    currentState = State.LIFT1;
                }
                break;

            case LIFT1:


                currentState = State.TO_PICKUP;
                follower.followPath(toPickup);


                break;


            case TO_PICKUP:

                if (!follower.isBusy()) {
                    currentState = State.INTAKE1;
                }
                break;

            case INTAKE1:


                currentState = State.TO_RUNG_2;
                follower.followPath(toRung2);
                break;

            case TO_RUNG_2:


                if (!follower.isBusy()) {
                    currentState = State.LIFT2;
                }
                break;

            case LIFT2:


                currentState = State.TO_OBSERVE;
                follower.followPath(toObserve);
                break;

            case TO_OBSERVE:

                if (!follower.isBusy()) {
                    currentState = State.IDLE;
                }
                break;

            case IDLE:
                // This concludes the autonomous program
                break;
        }
    }

    @Override
    public void loop() {

        double lastTime = 0;
        deltaTime = frameTimer.seconds();
        frameTimer.reset();

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        telemetry.addData("1", frameTimer.seconds());


        follower.update();
        telemetry.addData("2", frameTimer.seconds());


        autonomousPathUpdate();
        telemetry.addData("3", frameTimer.seconds());


        telemetry.addData("path state", currentState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("deltaTime", deltaTime);
        telemetry.addData("runTime", runtime);
        telemetry.addData("waitTime", time);
        telemetry.addData("4", frameTimer.seconds());
        telemetry.update();
    }


    @Override
    public void start() {
        buildPaths();
        deltaTime = 0;
        frameTimer.reset();
        currentState = State.TO_RUNG_START;
        follower.followPath(toRungStart);
    }

    @Override
    public void stop() {
    }
}

/**
 * 8==D
 */