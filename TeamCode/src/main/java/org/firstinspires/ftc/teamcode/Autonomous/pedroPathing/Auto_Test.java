package org.firstinspires.ftc.teamcode.Autonomous.pedroPathing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Pivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Lift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveIntake;

import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ReadOnlyRuntime;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import java.util.ArrayList;

@Autonomous(name = "SoupcOpMode_PP_Small_FSM-2")
public class Auto_Test extends OpMode{

    enum State {
        TO_RUNG_START,
        MOVE_SAMPLES,
        SCORE_SPECIMENS,
        IDLE            // Our bot will enter the IDLE state when done
    }

    State currentState = State.IDLE;

    //Points of Interest
    private final Pose startPose =        new Pose( 9,72, Math.toRadians(270));  // This is where the robot starts
    private final Pose rungpose =         new Pose(36,72, Math.toRadians(270));
    private final Pose rungpose1 =        new Pose(36,69, Math.toRadians(270));
    private final Pose rungpose2 =        new Pose(36,75, Math.toRadians(270));
    private final Pose rungpose3 =        new Pose(36,78, Math.toRadians(270));
    private final Pose rungposecurve =    new Pose(24,60, Math.toRadians(270));
    private final Pose interrimpose =     new Pose(36,36, Math.toRadians(180));
    private final Pose samplestartpose =  new Pose(60,36, Math.toRadians(180));
    private final Pose sample1pose =      new Pose(60,24, Math.toRadians(90));
    private final Pose sample2pose =      new Pose(60,24, Math.toRadians(90));
    private final Pose observe1pose =     new Pose(24,24, Math.toRadians(90));
    private final Pose observe2pose =     new Pose(24,12, Math.toRadians(90));
    private final Pose pickuppose =       new Pose(12,36, Math.toRadians(180));  // TODO: Make this more specific

    // List of paths the robot takes
    private Path toRungStart, toRung1, toRung2, toRung3, toPickup1, toPickup2;
    private PathChain moveSamples, scoreSpecimens;

    // Other misc. stuff
    private Follower follower;

    RobotConfig activeConfig = new RobotConfig(this);

    private final ReadOnlyRuntime runtime = new ReadOnlyRuntime();
    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;

    public void buildPaths() {
        toRungStart = new Path(new BezierLine (new Point(startPose),  new Point(rungpose)));
        toRung1 =   new Path(new BezierCurve  (new Point(pickuppose), new Point(rungposecurve), new Point(rungpose1)));
        toRung2 =   new Path(new BezierCurve  (new Point(pickuppose), new Point(rungposecurve), new Point(rungpose2)));
        toRung3 =   new Path(new BezierCurve  (new Point(pickuppose), new Point(rungposecurve), new Point(rungpose3)));
        toPickup1 = new Path(new BezierCurve  (new Point(rungpose1),  new Point(rungposecurve), new Point(pickuppose)));
        toPickup2 = new Path(new BezierCurve  (new Point(rungpose2),  new Point(rungposecurve), new Point(pickuppose)));

        moveSamples = follower.pathBuilder()
                .addPath(new BezierLine  (new Point(rungpose),        new Point(interrimpose)))
                .addPath(new BezierLine  (new Point(interrimpose),    new Point(samplestartpose)))
                .addPath(new BezierLine  (new Point(samplestartpose), new Point(sample1pose)))
                .addPath(new BezierLine  (new Point(sample1pose),     new Point(observe1pose)))
                .addPath(new BezierLine  (new Point(observe1pose),    new Point(sample1pose)))
                .addPath(new BezierLine  (new Point(sample1pose),     new Point(sample2pose)))
                .addPath(new BezierLine  (new Point(sample2pose),     new Point(observe2pose)))
                .addPath(new BezierCurve (new Point(observe2pose),    new Point(observe1pose), new Point(pickuppose)))
                .build();

        scoreSpecimens = follower.pathBuilder()
                .addPath(toRung1)
                .addPath(toPickup1)
                .addPath(toRung2)
                .addPath(toPickup2)
                .addPath(toRung3)
                .build();
    }

    public void autonomousPathUpdate() {

        switch (currentState) {

            case TO_RUNG_START:
                telemetry.addLine("It work 1");
                telemetry.update();
                // Check if the drive class is busy turning
                // If not, move onto the next state
                // We are done with the program
                if (!follower.isBusy()) {
                    currentState = State.MOVE_SAMPLES;
                    follower.followPath(moveSamples);
                }
                break;

            case MOVE_SAMPLES:
                // Check if the drive class is busy turning
                // If not, move onto the next state
                // We are done with the program
                if (!follower.isBusy()) {

                    currentState = State.SCORE_SPECIMENS;
                    follower.followPath(scoreSpecimens);
                }
                break;

            case SCORE_SPECIMENS:
                // Check if the drive class is busy turning
                // If not, move onto the next state
                // We are done with the program
                if (!follower.isBusy()) {

                    currentState = State.IDLE;
                }
                break;

            case IDLE:
                telemetry.addLine("It work 2");
                telemetry.update();
                // Do nothing in IDLE
                // currentState does not change once in IDLE
                // This concludes the autonomous program
                break;
        }
    }

    @Override
    public void loop() {
        follower.update();

        autonomousPathUpdate();

        telemetry.addData("path state", currentState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize our lift
        RobotConfig activeConfig = new RobotConfig(this);

        Lift lift = new Lift(this, activeConfig, runtime);

        lift.setControlMode(ControlAxis.ControlMode.positionControl);

        Pivot spinyBit = new Pivot(this, activeConfig, runtime);

        spinyBit.setControlMode(ControlAxis.ControlMode.positionControl);

        //Pincher pincher = new Pincher(this,activeConfig);

        ActiveIntake intake = new ActiveIntake(this, activeConfig);


        RobotConfig config = new RobotConfig(this);
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