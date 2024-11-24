package org.firstinspires.ftc.teamcode.automous_OpModes;


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
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveIntake;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ReadOnlyRuntime;

@Autonomous(name = "SoupcOpMode_0-4-0 0.1.0")
public class Auto_Test_040 extends OpMode{

    enum State {
        TO_RUNG_START,
        MOVE_SAMPLES,
        MOVE_SAMPLES2,
        SCORE_SPECIMENS,
        IDLE            // Our bot will enter the IDLE state when done
    }

    State currentState = State.IDLE;

    //Points of Interest
    private final Pose startPose =        new Pose( 9,72, Math.toRadians(0));  // This is where the robot starts
    private final Pose rungpose =         new Pose(36,72, Math.toRadians(0));
    private final Pose rungpose1 =        new Pose(36,69, Math.toRadians(0));
    private final Pose rungpose2 =        new Pose(36,75, Math.toRadians(0));
    private final Pose rungpose3 =        new Pose(36,78, Math.toRadians(0));
    private final Pose rungposecurve =    new Pose(24,60, Math.toRadians(0));
    private final Pose interrimpose =     new Pose(36,38, Math.toRadians(0));
    private final Pose samplestartpose =  new Pose(60,38, Math.toRadians(0));
    private final Pose sample1pose =      new Pose(60,24, Math.toRadians(0));
    private final Pose sample2pose =      new Pose(60,24, Math.toRadians(0));
    private final Pose observe1pose =     new Pose(24,24, Math.toRadians(0));
    private final Pose observe2pose =     new Pose(24,12, Math.toRadians(0));
    private final Pose pickuppose =       new Pose(12,38, Math.toRadians(270));  // TODO: Make this more specific

    // List of paths the robot takes
    private Path toRungStart, toRung1, toRung2, toRung3, toPickup1, toPickup2;
    private PathChain moveSamples, scoreSpecimens, moveSamples2;

    // Other misc. stuff
    private Follower follower;

    RobotConfig activeConfig = new RobotConfig(this);

    private final ReadOnlyRuntime runtime = new ReadOnlyRuntime();
    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;

    public void buildPaths() {
        toRungStart = new Path(new BezierLine (new Point(startPose),  new Point(rungpose)));
        toRungStart.setConstantHeadingInterpolation(Math.toRadians(0));
        toRung1 =   new Path(new BezierCurve  (new Point(pickuppose), new Point(rungposecurve), new Point(rungpose1)));
        toRung2 =   new Path(new BezierCurve  (new Point(pickuppose), new Point(rungposecurve), new Point(rungpose2)));
        toRung3 =   new Path(new BezierCurve  (new Point(pickuppose), new Point(rungposecurve), new Point(rungpose3)));
        toPickup1 = new Path(new BezierCurve  (new Point(rungpose1),  new Point(rungposecurve), new Point(pickuppose)));
        toPickup2 = new Path(new BezierCurve  (new Point(rungpose2),  new Point(rungposecurve), new Point(pickuppose)));

        moveSamples = follower.pathBuilder()
                .addPath(new BezierLine  (new Point(rungpose),        new Point(interrimpose)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .addPath(new BezierLine  (new Point(interrimpose),    new Point(samplestartpose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine  (new Point(samplestartpose), new Point(sample1pose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine  (new Point(sample1pose),     new Point(observe1pose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine  (new Point(observe1pose),    new Point(sample1pose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        moveSamples2 = follower.pathBuilder()
                .addPath(new BezierLine  (new Point(sample1pose),     new Point(sample2pose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine  (new Point(sample2pose),     new Point(observe2pose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve (new Point(observe2pose),    new Point(observe1pose), new Point(pickuppose)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                .build();

        scoreSpecimens = follower.pathBuilder()
                .addPath(toRung1)
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                .addPath(toPickup1)
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                .addPath(toRung2)
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                .addPath(toPickup2)
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                .addPath(toRung3)
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {

        switch (currentState) {

            case TO_RUNG_START:

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

                    currentState = State.MOVE_SAMPLES2;
                    follower.followPath(moveSamples2);
                }
                break;

            case MOVE_SAMPLES2:
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

        //Lift lift = new Lift(this, activeConfig, runtime);

        //lift.setControlMode(ControlAxis.ControlMode.positionControl);

        //Pivot spinyBit = new Pivot(this, activeConfig, runtime);

        //spinyBit.setControlMode(ControlAxis.ControlMode.positionControl);

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