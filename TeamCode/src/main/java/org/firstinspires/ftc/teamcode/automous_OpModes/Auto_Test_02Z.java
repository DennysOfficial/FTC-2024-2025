package org.firstinspires.ftc.teamcode.automous_OpModes;


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

@Autonomous(name = "SoupcOpMode_0-2-Z 0.0.0")
public class Auto_Test_02Z extends OpMode{

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

    private final Pose startPose = new Pose(2,72, Math.toRadians(0));  // This is where the robot starts

    //Points of Interest
    private final Point rungpoint =    new Point(39,72, Point.CARTESIAN);
    private final Point rungpoint1 =   new Point(39,69, Point.CARTESIAN);
    private final Point curvepoint =   new Point(12,72, Point.CARTESIAN);
    private final Point observepoint = new Point( 9, 9, Point.CARTESIAN);
    private final Point pickuppoint =  new Point(15.5,41, Point.CARTESIAN); // TODO: Make this more specific


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
        config = new RobotConfig(this);

        lift = new Lift(ControlAxis.ControlMode.positionControl,this, config, runtime);
        spinyBit = new Pivot(ControlAxis.ControlMode.positionControl,this, config, runtime);
        intake = new ActiveIntake(this, config);

        lift.assignPivot(spinyBit);
        spinyBit.assignLift(lift);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
    }



    public void buildPaths() {

        toRungStart = follower.pathBuilder()
                .addPath(new BezierLine (new Point(startPose), rungpoint))
                .addTemporalCallback(0, () -> {
                    spinyBit.setTargetPosition(0);
                    lift.setTargetPosition(9.5);
                })
                .build();


        toPickup = follower.pathBuilder()
                .addPath(new BezierCurve(rungpoint, curvepoint, pickuppoint))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                .build();


        toRung2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickuppoint, curvepoint, rungpoint1))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                .addTemporalCallback(0, () -> {
                    spinyBit.setTargetPosition(0);
                    lift.setTargetPosition(9.5);
                })
                .build();


        toObserve = follower.pathBuilder()
                .addPath(new BezierCurve(rungpoint1, curvepoint, observepoint))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {

        switch (currentState) {

            case TO_RUNG_START:

                // Check if the drive class is busy turning
                // If not, move onto the next state
                // We are done with the program
                if (!follower.isBusy()) {
                    currentState = State.LIFT1;
                }
                break;

            case LIFT1:
                lift.setTargetPosition(0);
                if (lift.getPosition() <= 1) {
                    currentState = State.TO_PICKUP;
                    follower.followPath(toPickup);

                }
                break;


            case TO_PICKUP:
                // Check if the drive class is busy turning
                // If not, move onto the next state
                // We are done with the program
                if (!follower.isBusy()) {
                    currentState = State.INTAKE1;
                }
                break;

            case INTAKE1:

                spinyBit.setTargetPosition(90);
                intake.intake();
                double time = runtime.seconds();

                if (time + 0.5 <= runtime.seconds() && spinyBit.getPosition() == 90) {
                    intake.stop();
                    currentState = State.TO_RUNG_2;
                    follower.followPath(toRung2);
                }
                break;

            case TO_RUNG_2:
                // Check if the drive class is busy turning
                // If not, move onto the next state
                // We are done with the program
                if (!follower.isBusy()) {
                    currentState = State.LIFT2;
                }
                break;

            case LIFT2:
                lift.setTargetPosition(0);

                if (lift.getPosition() <= 1) {
                    currentState = State.TO_OBSERVE;
                    follower.followPath(toObserve);
                }
                break;

            case TO_OBSERVE:
                // Check if the drive class is busy turning
                // If not, move onto the next state
                // We are done with the program
                if (!follower.isBusy()) {

                    currentState = State.IDLE;
                }
                break;

            case IDLE:
                // Do nothing in IDLE
                // currentState does not change once in IDLE
                // This concludes the autonomous program
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();
        lift.update();
        spinyBit.update();

        autonomousPathUpdate();

        telemetry.addData("path state", currentState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
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