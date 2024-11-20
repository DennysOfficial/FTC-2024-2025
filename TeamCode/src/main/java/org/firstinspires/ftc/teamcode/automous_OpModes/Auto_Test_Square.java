package org.firstinspires.ftc.teamcode.automous_OpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Lift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Pivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveIntake;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ReadOnlyRuntime;

@Autonomous(name = "SquareTest")
public class Auto_Test_Square extends OpMode{

    enum State {
        MOVE,
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
    private PathChain moveSamples, scoreSpecimens, square;

    // Other misc. stuff
    private Follower follower;

    RobotConfig activeConfig = new RobotConfig(this);

    private final ReadOnlyRuntime runtime = new ReadOnlyRuntime();
    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;

    public void buildPaths() {
        square = follower.pathBuilder()
                .addPath(new BezierLine(new Point(0,0, Point.CARTESIAN),new Point(0,10, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(0,10, Point.CARTESIAN),new Point(10,10, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(10,10, Point.CARTESIAN),new Point(10,0, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(new Point(10,0, Point.CARTESIAN),new Point(0,0, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {

        switch (currentState) {

            case MOVE:

                // Check if the drive class is busy turning
                // If not, move onto the next state
                // We are done with the program
                if (!follower.isBusy()) {
                    currentState = State.IDLE;
                    follower.holdPoint(new BezierPoint(new Point(0, 0, Point.CARTESIAN)), Math.toRadians(0));
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
        follower.setStartingPose(new Pose(0,0, Math.toRadians(0)));

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
        currentState = State.MOVE;
        follower.followPath(square);
    }

    @Override
    public void stop() {
    }
}

/**
 * 8==D
 */