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
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.List;

@Autonomous(name = "SoupcOpMode_4-0-Z 0.0.0")
public class Auto_Test_40Z extends OpMode{


    enum State {
        TO_BASKET1,
        TO_BASKET2,
        TO_BASKET3,
        TO_BASKET4,
        TO_SPIKE1,
        TO_SPIKE2,
        TO_SPIKE3,
        TO_BAR,
        IDLE
    }

    State currentState = State.IDLE;

    List<LynxModule> allHubs;

    private final Pose startPose = new Pose(9,108, Math.toRadians(270));  // This is where the robot starts

    //Points of Interest
    private Point spikepoint1 =        new Point(30,124, Point.CARTESIAN);
    private Point spikepoint2 =        new Point(30,135, Point.CARTESIAN);
    private Point spikepoint3 =        new Point(66,132, Point.CARTESIAN);

    private Point basketpoint =        new Point(9,112, Point.CARTESIAN);

    private Point barpoint =           new Point(60,98, Point.CARTESIAN);

    private Point barcurvepoint =      new Point(72,120, Point.CARTESIAN);


    public Path toBasket1, toBasket2, toBasket3, toBasket4, toSpike1, toSpike2, toSpike3, toBar;

    // Other misc. stuff
    private Follower follower;

    double liftPosBasket = 33;
    double PIVOT_POS = -10;
    double pivotPosSpike = 90;

    private final ReadOnlyRuntime runtime = new ReadOnlyRuntime();
    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;

    Lift lift;
    Pivot pivot;
    ActiveIntake intake;
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

        lift = new Lift(ControlAxis.ControlMode.positionControl,this, config);
        pivot = new Pivot(ControlAxis.ControlMode.positionControl,this, config);
        intake = new ActiveIntake(this, config);

        lift.assignPivot(pivot);
        pivot.assignLift(lift);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
    }



    public void buildPaths() {

        toBasket1 = new Path(new BezierLine(new Point(startPose), basketpoint));
        toBasket2 = new Path(new BezierLine(spikepoint1, basketpoint));
        toBasket3 = new Path(new BezierLine(spikepoint2, basketpoint));
        toBasket4 = new Path(new BezierLine(spikepoint3, basketpoint));

        toSpike1 = new Path(new BezierLine(basketpoint, spikepoint1));
        toSpike2 = new Path(new BezierLine(basketpoint, spikepoint2));
        toSpike3 = new Path(new BezierLine(basketpoint, spikepoint3));

        toBar = new Path(new BezierCurve(basketpoint, barcurvepoint, barpoint));


        toBar.setConstantHeadingInterpolation(270);

        toBasket1.setConstantHeadingInterpolation(270);

        toBasket2.setLinearHeadingInterpolation(0, 270);
        toBasket3.setLinearHeadingInterpolation(0, 270);
        toBasket4.setLinearHeadingInterpolation(0, 270);

        toSpike1.setLinearHeadingInterpolation(270, 0);
        toSpike2.setLinearHeadingInterpolation(270, 0);
        toSpike3.setLinearHeadingInterpolation(270, 0);
    }

    public void autonomousPathUpdate() {

        switch (currentState) {

            case TO_BASKET1:
                if (follower.atParametricEnd() && lift.getPosition() >= 32.5) {
                    pivot.setTargetPosition(PIVOT_POS);
                        if(pivot.getPosition() <= -9.7) {
                            intake.openFlap();
                            pathTimer.resetTimer();
                            intake.intakeForDuration(0.5);
                                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                                    intake.closeFlap();
                                    follower.followPath(toSpike1);
                                    currentState = State.TO_SPIKE1;
                                    pivot.fancyMoveToPosition(80, 2);
                                    lift.setTargetPosition(6);
                                    pathTimer.resetTimer();
                                }
                        }
                }
                break;

            case TO_SPIKE1:
                if (follower.atParametricEnd()) {
                    pivot.setTargetPosition(90);
                    if (pivot.getPosition() >= 87.5 || pathTimer.getElapsedTimeSeconds() >= 3.5) {
                        intake.intakeForDuration(0.5);
                        pathTimer.resetTimer();
                        follower.followPath(toBasket2);
                        currentState = State.TO_BASKET2;
                        lift.setTargetPosition(liftPosBasket);
                        pivot.setTargetPosition(0);
                    }
                }
                break;

            case TO_BASKET2:
                if (follower.atParametricEnd() && lift.getPosition() >= 32.5) {
                    pivot.setTargetPosition(PIVOT_POS);
                    if(pivot.getPosition() <= -9.7) {
                        intake.openFlap();
                        pathTimer.resetTimer();
                        intake.intakeForDuration(0.5);
                        if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                            intake.closeFlap();
                            follower.followPath(toSpike2);
                            currentState = State.TO_SPIKE2;
                            pivot.fancyMoveToPosition(80, 2);
                            lift.setTargetPosition(6);
                            pathTimer.resetTimer();
                        }
                    }
                }
                break;

            case TO_SPIKE2:
                if (follower.atParametricEnd()) {
                    pivot.setTargetPosition(90);
                    if (pivot.getPosition() >= 87.5 || pathTimer.getElapsedTimeSeconds() >= 3.5) {
                        intake.intakeForDuration(0.5);
                        pathTimer.resetTimer();
                        follower.followPath(toBasket3);
                        currentState = State.TO_BASKET3;
                        lift.setTargetPosition(liftPosBasket);
                        pivot.setTargetPosition(0);
                    }
                }
                break;

            case TO_BASKET3:
                if (follower.atParametricEnd() && lift.getPosition() >= 32.5) {
                    pivot.setTargetPosition(PIVOT_POS);
                    if(pivot.getPosition() <= -9.7) {
                        intake.openFlap();
                        pathTimer.resetTimer();
                        intake.intakeForDuration(0.5);
                        if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                            intake.closeFlap();
                            follower.followPath(toSpike3);
                            currentState = State.TO_SPIKE3;
                            pivot.fancyMoveToPosition(80, 2);
                            lift.setTargetPosition(6);
                            pathTimer.resetTimer();
                        }
                    }
                }
                break;
            case TO_SPIKE3:
                if (follower.atParametricEnd()) {
                    pivot.setTargetPosition(90);
                    if (pivot.getPosition() >= 87.5 || pathTimer.getElapsedTimeSeconds() >= 3.5) {
                        intake.intakeForDuration(0.5);
                        pathTimer.resetTimer();
                        follower.followPath(toBasket4);
                        currentState = State.TO_BASKET4;
                        lift.setTargetPosition(liftPosBasket);
                        pivot.setTargetPosition(0);
                    }
                }
                break;

            case TO_BASKET4:
                if (follower.atParametricEnd() && lift.getPosition() >= 32.5) {
                    pivot.setTargetPosition(PIVOT_POS);
                    if(pivot.getPosition() <= -9.7) {
                        intake.openFlap();
                        pathTimer.resetTimer();
                        intake.intakeForDuration(0.5);
                        if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                            intake.closeFlap();
                            follower.followPath(toBar);
                            currentState = State.TO_BAR;
                            pivot.fancyMoveToPosition(0, 0.75);
                            lift.setTargetPosition(12);
                            pathTimer.resetTimer();
                        }
                    }
                }
                break;

            case TO_BAR:
                if (follower.atParametricEnd() || pathTimer.getElapsedTimeSeconds() >= 4)  {
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
        pathTimer.resetTimer();

        lift.setTargetPosition(liftPosBasket);
        pivot.fancyMoveToPosition(0, 0.75);

        currentState = State.TO_BASKET1;
        follower.followPath(toBasket1);
    }

    @Override
    public void stop() {
    }
}

/**
 * 8==D
 */