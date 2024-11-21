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
        IDLE            // Our bot will enter the IDLE state when done
    }

    State currentState = State.IDLE;

    private final Pose startPose = new Pose(9,72, Math.toRadians(0));  // This is where the robot starts

    //Points of Interest
    private final Point rungpoint =    new Point(39,72, Point.CARTESIAN);
    private final Point rungpoint1 =   new Point(39,69, Point.CARTESIAN);
    private final Point curvepoint =   new Point(12,72, Point.CARTESIAN);
    private final Point observepoint = new Point( 9, 9, Point.CARTESIAN);
    private final Point pickuppoint =  new Point(12,36, Point.CARTESIAN);// TODO: Make this more specific


    // List of paths the robot takes
    private PathChain toRungStart, toPickup, toRung2, toObserve;

    // Other misc. stuff
    private Follower follower;

    RobotConfig config;
    Pivot spinyBit;
    Lift lift;
    ActiveIntake intake;

    private final ReadOnlyRuntime runtime = new ReadOnlyRuntime();
    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize our lift
        RobotConfig activeConfig = new RobotConfig(this);

        lift = new Lift(this, activeConfig, runtime);

        lift.setControlMode(ControlAxis.ControlMode.positionControl);

        spinyBit = new Pivot(this, activeConfig, runtime);

        spinyBit.setControlMode(ControlAxis.ControlMode.positionControl);

        //Pincher pincher = new Pincher(this,activeConfig);

        intake = new ActiveIntake(this, activeConfig);

        config = new RobotConfig(this);

        lift.assignPivot(spinyBit);
        spinyBit.assignLift(lift);
    }



    public void buildPaths() {

        toRungStart = follower.pathBuilder()
                .addPath(new BezierLine (new Point(startPose), rungpoint))
                .addParametricCallback(0, () -> {
                    lift.setTargetPosition(8.8);
                    spinyBit.setTargetPosition(0);
                })
                .addParametricCallback(1, () -> {
                    lift.setTargetPosition(0);
                })
                .build();


        toPickup = follower.pathBuilder()
                .addPath(new BezierCurve(rungpoint, curvepoint, pickuppoint))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                .addParametricCallback(1, () -> {
                    spinyBit.setTargetPosition(80);
                    //TODO: Pick up specimen, idk I just work here
                })
                .build();


        toRung2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickuppoint, curvepoint, rungpoint1))
                .addParametricCallback(0, () -> {
                    lift.setTargetPosition(8.8);
                    spinyBit.setTargetPosition(0);
                })
                .addParametricCallback(1, () -> {
                    lift.setTargetPosition(0);
                })
                .build();


        toObserve = follower.pathBuilder()
                .addPath(new BezierCurve(rungpoint1, curvepoint, observepoint))
                .build();
    }

    public void autonomousPathUpdate() {

        switch (currentState) {

            case TO_RUNG_START:

                // Check if the drive class is busy turning
                // If not, move onto the next state
                // We are done with the program
                if (!follower.isBusy()) {
                    currentState = State.IDLE;
                    //follower.followPath(toPickup);
                }
                break;

            case TO_PICKUP:
                // Check if the drive class is busy turning
                // If not, move onto the next state
                // We are done with the program
                if (!follower.isBusy()) {

                    currentState = State.TO_RUNG_2;
                    follower.followPath(toRung2);
                }
                break;

            case TO_RUNG_2:
                // Check if the drive class is busy turning
                // If not, move onto the next state
                // We are done with the program
                if (!follower.isBusy()) {

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
        //lift.update();
        //spinyBit.update();

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