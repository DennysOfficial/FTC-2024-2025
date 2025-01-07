package org.firstinspires.ftc.teamcode.automous_OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Lift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Pivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveIntake;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ReadOnlyRuntime;
import org.firstinspires.ftc.teamcode.pedroPathing.Automous;
import org.firstinspires.ftc.teamcode.pedroPathing.LiftTimeStamp;
import org.firstinspires.ftc.teamcode.pedroPathing.TimeStamp;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.List;

@Autonomous(name = "SoupcOpMode_0-5-Z 2.0.0")
public class Auto_Test_05Z extends OpMode{


    enum State {
        TO_RUNG1,
        TO_RUNG2,
        TO_RUNG3,
        TO_RUNG4,
        TO_RUNG5,
        TO_PICKUP2,
        TO_PICKUP3,
        TO_PICKUP4,
        TO_OBSERVE,
        MOVE_SAMPLES,
        IDLE
    }

    State currentState = State.IDLE;

    List<LynxModule> allHubs;

    private final Pose startPose = new Pose(9,64.5, Math.toRadians(0));  // This is where the robot starts

    //Points of Interest
    Point rungPoint1 = new Point(39, 64.5, Point.CARTESIAN);
    Point rungPoint2 = new Point(39, 66, Point.CARTESIAN);
    Point rungPoint3 = new Point(39, 67.5, Point.CARTESIAN);
    Point rungPoint4 = new Point(39, 69, Point.CARTESIAN);
    Point rungPoint5 = new Point(39, 70.5, Point.CARTESIAN);

    Point pickupPoint1 = new Point(9, 12, Point.CARTESIAN);
    Point pickupPoint2 = new Point(9, 36, Point.CARTESIAN);

    Point samplePoint1 = new Point(24, 24, Point.CARTESIAN);
    Point samplePoint2 = new Point(24, 12, Point.CARTESIAN);

    Point parkPoint = new Point(24, 48, Point.CARTESIAN);


    public PathChain toRung1, toRung2, toRung3, toRung4, toRung5, toSample1, toSample2, toSample3, toPickup1, toPickup2, toPickup3, toPickup4, toPark;

    // Other misc. stuff
    private Follower follower;

    double liftPosRung = 13.5;
    double pivotPosRung = 25;

    double pivotPosObs = 90;

    private final ReadOnlyRuntime runtime = new ReadOnlyRuntime();
    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;

    Lift lift;
    Pivot pivot;
    ActiveIntake intake;
    RobotConfig config;

    Automous automous;

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

        automous = new Automous(this, lift, pivot, intake, config, follower);
    }



    public void buildPaths() {
        toRung1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(startPose), rungPoint1)))
                .build();
        toRung2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickupPoint1, rungPoint2)))
                .setConstantHeadingInterpolation(0)
                .build();
        toRung3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickupPoint2, rungPoint3)))
                .setConstantHeadingInterpolation(0)
                .build();
        toRung4 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickupPoint2, rungPoint4)))
                .setConstantHeadingInterpolation(0)
                .build();
        toRung5 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickupPoint2, rungPoint5)))
                .setConstantHeadingInterpolation(0)
                .build();


        toSample1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(rungPoint1, samplePoint1)))
                .setConstantHeadingInterpolation(0)
                .build();
        toSample2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(samplePoint1,samplePoint2)))
                .setConstantHeadingInterpolation(0)
                .build();
        toSample3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(samplePoint2,samplePoint2)))
                .setLinearHeadingInterpolation(0, 340)
                .build();


        toPickup1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(samplePoint2, pickupPoint1)))
                .setLinearHeadingInterpolation(340, 0)
                .build();
        toPickup2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(rungPoint2, pickupPoint2)))
                .setConstantHeadingInterpolation(0)
                .build();
        toPickup3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(rungPoint3, pickupPoint2)))
                .setConstantHeadingInterpolation(0)
                .build();
        toPickup4 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(rungPoint4, pickupPoint2)))
                .setConstantHeadingInterpolation(0)
                .build();


        toPark = follower.pathBuilder()
                .addPath(new Path(new BezierLine(rungPoint5, parkPoint)))
                .setLinearHeadingInterpolation(0, 240)
                .build();

        //TODO: Lift values are ESTIMATES, any comments in this codeblock are spots where lift code is needed
        automous.addPath(25, 10, toRung1, 0, 0, 5); //1
        automous.addPath(0, 0, toSample1, 90, 10, 5); //2
        automous.addPath(90, 10, null, 0, 0, 5); //3
        automous.addTimeStamp(new TimeStamp(() -> {
            //intake go brr
        }, 1, 3));
        automous.addTimeStamp(new TimeStamp(() -> {
            //intake go spit
        }, 5, 3));
        automous.addPath(0, 0, toSample2, 90, 10, 5); //4
        automous.addPath(90, 10, null, 0, 0, 5); //5
        automous.addTimeStamp(new TimeStamp(() -> {
            //intake go brr
        }, 1, 5));
        automous.addTimeStamp(new TimeStamp(() -> {
            //intake go spit
        }, 5, 5));
        automous.addPath(0, 0, toSample3, 90, 10, 5); //6
        automous.addPath(90, 10, null, 0, 0, 5); //7
        automous.addTimeStamp(new TimeStamp(() -> {
            //intake go brr
        }, 1, 7));
        automous.addTimeStamp(new TimeStamp(() -> {
            //intake go spit
        }, 5, 7));
        automous.addPath(0, 0, toPickup1, 0, 0, 5); //8
        automous.addPath(0, 0, toRung2, 0, 0, 5); //9
        automous.addLiftTimeStamp(new LiftTimeStamp(25, 10, 0.5, 9));
        automous.addPath(0, 0, toPickup2, 0, 0, 5);//10
        automous.addPath(0, 0, toRung3, 0, 0, 5); //11
        automous.addLiftTimeStamp(new LiftTimeStamp(25, 10, 0.5, 11));
        automous.addPath(0, 0, toPickup3, 0, 0, 5);//12
        automous.addPath(0, 0, toRung4, 0, 0, 5); //13
        automous.addLiftTimeStamp(new LiftTimeStamp(25, 10, 0.5, 13));
        automous.addPath(0, 0, toPickup4, 0, 0, 5);//14
        automous.addPath(0, 0, toRung5, 0, 0, 5); //15
        automous.addLiftTimeStamp(new LiftTimeStamp(25, 10, 0.5, 15));
        automous.addPath(0, 0, toPark, 90, 33, 5);//16
        automous.addLiftTimeStamp(new LiftTimeStamp(90, 33, 1, 16));
    }

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        deltaTime = frameTimer.seconds();
        frameTimer.reset();

        intake.closeFlap();

        automous.routine();
        automous.update();

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
    }

    @Override
    public void stop() {
    }
}

/**
 * 8==D
 */