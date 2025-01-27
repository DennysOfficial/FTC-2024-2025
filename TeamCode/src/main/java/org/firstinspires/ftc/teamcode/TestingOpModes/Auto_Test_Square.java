package org.firstinspires.ftc.teamcode.TestingOpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftPivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveIntakeServo;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;

import java.util.List;

@Autonomous(name = "SquareTest")
public class Auto_Test_Square extends OpMode{

    enum State {
        MOVE,
        IDLE            // Our bot will enter the IDLE state when done
    }

    State currentState = State.IDLE;

    LeftLift lift;
    LeftPivot spinyBit;
    ActiveIntakeServo intake;

    RobotConfig config;

    List<LynxModule> allHubs;

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
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        deltaTime = frameTimer.seconds();
        frameTimer.reset();

        follower.update();
        lift.update();
        spinyBit.update();
        intake.update();

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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk


        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        config = new RobotConfig(this);

        lift = new LeftLift(ControlAxis.ControlMode.positionControl,this, config);
        spinyBit = new LeftPivot(ControlAxis.ControlMode.positionControl,this, config);
        intake = new ActiveIntakeServo(this, config);

        lift.assignPivot(spinyBit);
        spinyBit.assignLift(lift);
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