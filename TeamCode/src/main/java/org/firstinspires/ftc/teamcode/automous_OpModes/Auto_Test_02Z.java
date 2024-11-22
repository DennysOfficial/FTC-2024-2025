package org.firstinspires.ftc.teamcode.automous_OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Pivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Lift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveIntake;

import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ReadOnlyRuntime;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.List;

@Autonomous(name = "SoupcOpMode_0-2-Z 1.0.2")
public class Auto_Test_02Z extends OpMode {

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

    Lift lift;
    Pivot spinyBit;
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

        lift = new Lift(ControlAxis.ControlMode.positionControl, this, config, runtime);
        spinyBit = new Pivot(ControlAxis.ControlMode.positionControl, this, config, runtime);
        intake = new ActiveIntake(this, config);

        lift.assignPivot(spinyBit);
        spinyBit.assignLift(lift);

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
                    lift.setTargetPosition(0);
                }
                break;

            case LIFT1:

                if (lift.getPosition() <= 1) {
                    currentState = State.TO_PICKUP;
                    follower.followPath(toPickup);

                }
                break;


            case TO_PICKUP:

                if (!follower.isBusy()) {
                    currentState = State.INTAKE1;
                    spinyBit.fancyMoveToPosition(90, 1);
                    time = runtime.seconds() + 0.5;
                }
                break;

            case INTAKE1:

                if (spinyBit.getPosition() >= 87.5) {
                    intake.intake();

                    if (time <= runtime.seconds()) {

                        intake.stop();
                        currentState = State.TO_RUNG_2;
                        follower.followPath(toRung2);

                        spinyBit.fancyMoveToPosition(5, 1.5);
                        lift.fancyMoveToPosition(9.5, 1.5);
                    }
                }
                break;

            case TO_RUNG_2:

                if (!follower.isBusy()) {
                    currentState = State.LIFT2;
                    lift.setTargetPosition(0);
                }
                break;

            case LIFT2:

                if (lift.getPosition() <= 1) {
                    currentState = State.TO_OBSERVE;
                    follower.followPath(toObserve);
                }
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
        lift.update();
        telemetry.addData("3", frameTimer.seconds());
        spinyBit.update();
        telemetry.addData("4", frameTimer.seconds());

        autonomousPathUpdate();
        telemetry.addData("5", frameTimer.seconds());

        telemetry.addData("path state", currentState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("deltaTime", deltaTime);
        telemetry.addData("runTime", runtime);
        telemetry.addData("waitTime", time);
        telemetry.addData("6", frameTimer.seconds());
        telemetry.update();
    }


    @Override
    public void start() {
        buildPaths();
        deltaTime = 0;
        frameTimer.reset();
        currentState = State.TO_RUNG_START;
        follower.followPath(toRungStart);

        spinyBit.fancyMoveToPosition(5, 1.5);
        lift.fancyMoveToPosition(9.5, 1.5);
    }

    @Override
    public void stop() {
    }
}

/**
 * 8==D
 */