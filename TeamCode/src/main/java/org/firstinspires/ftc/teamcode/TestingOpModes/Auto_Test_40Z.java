package org.firstinspires.ftc.teamcode.TestingOpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftPivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.RightLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.RightPivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveIntakeMotor;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveIntakeServo;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.PassiveGrabber;
import org.firstinspires.ftc.teamcode.pedroPathing.Automous;
import org.firstinspires.ftc.teamcode.pedroPathing.AutomousNoLift;
import org.firstinspires.ftc.teamcode.pedroPathing.LiftTimeStamp;
import org.firstinspires.ftc.teamcode.pedroPathing.TimeStamp;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.List;

@Autonomous(name = "SoupcOpMode_4-0-Z 0.0.1")
public class Auto_Test_40Z extends OpMode{


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

    private final Pose startPose = new Pose(9,108, Math.toRadians(0));  // This is where the robot starts

    //Points of Interest
    public Point scorePoint = new Point(9, 120, Point.CARTESIAN);

    public Point samplePoint1 = new Point(24, 120, Point.CARTESIAN);
    public Point samplePoint2 = new Point(24, 132, Point.CARTESIAN);

    public Point barPoint = new Point(60, 96, Point.CARTESIAN);
    public Point barPointControl = new Point(72, 120, Point.CARTESIAN);

    public PathChain toScore1, toSample1, toScore2, toSample2, toScore3, toSample3, toScore4, toBar;

    // Other misc. stuff
    private Follower follower;

    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;

    LeftLift leftLift;
    LeftPivot leftPivot;
    ActiveIntakeMotor intake;

    RightLift rightLift;
    RightPivot rightPivot;
    PassiveGrabber grabber;

    RobotConfig config;

    AutomousNoLift automous; //CHANGE THIS WHEN NEW ROBOT

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk


        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        config = new RobotConfig(this);

        leftLift = new LeftLift(ControlAxis.ControlMode.positionControl,this, config);
        leftPivot = new LeftPivot(ControlAxis.ControlMode.positionControl,this, config);
        intake = new ActiveIntakeMotor(this, config);

        rightLift = new RightLift(ControlAxis.ControlMode.positionControl,this, config);
        rightPivot = new RightPivot(ControlAxis.ControlMode.positionControl,this, config);
        grabber = new PassiveGrabber(this, config, leftLift, leftPivot);

        leftLift.assignPivot(leftPivot);
        leftPivot.assignLift(leftLift);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        automous = new AutomousNoLift(this, config, follower);
    }



    public void buildPaths() {

        toScore1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), scorePoint))
                .setConstantHeadingInterpolation(270)
                .build();
        toSample1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePoint, samplePoint1))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                .build();
        toScore2 = follower.pathBuilder()
                .addPath(new BezierLine(samplePoint1, scorePoint))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                .build();
        toSample2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePoint, samplePoint2))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                .build();
        toScore3 = follower.pathBuilder()
                .addPath(new BezierLine(samplePoint2, scorePoint))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                .build();
        toSample3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePoint, samplePoint2))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                .build();
        toScore4 = follower.pathBuilder()
                .addPath(new BezierLine(samplePoint2, scorePoint))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                .build();
        toBar = follower.pathBuilder()
                .addPath(new BezierCurve(scorePoint, barPointControl, barPoint))
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();
        //TODO: Lift values are ESTIMATES

        automous.addPath(-10, 33, toScore1, 0, 2); //1 - Score first sample

        automous.addLiftTimeStamp(new LiftTimeStamp(-10, 33, 0, 1)); //Put arm in scoring position


        automous.addPath(90, 5, toSample1, 0, 3); //2 - Collect second sample


        automous.addPath(0, 0, null, 0, 3); //3 - Move lift to collect and intake

        automous.addTimeStamp(new TimeStamp(() -> { //Intake for 3 seconds
            intake.intakeForDuration(3);
        }, 0, 3));


        automous.addPath(-10, 33, toScore2, 0, 3); //4 - Score second sample

        automous.addLiftTimeStamp(new LiftTimeStamp(-10, 33, 1, 4)); //Put arm in scoring position


        automous.addPath(90, 5, toSample2, 0, 3); //5 - Collect third sample


        automous.addPath(0, 0, null, 0, 3); //6 - Move lift to collect and intake

        automous.addTimeStamp(new TimeStamp(() -> { //Intake for 3 seconds
            intake.intakeForDuration(3);
        }, 0, 6));


        automous.addPath(-10, 33, toScore3, 0, 3); //7 - Score third sample

        automous.addLiftTimeStamp(new LiftTimeStamp(-10, 33, 1, 7)); //Put arm in scoring position


        automous.addPath(90, 5, toSample3, 0, 3); //8 - Collect fourth sample


        automous.addPath(0, 0, null, 0, 3); //9 - Move lift to collect and intake

        automous.addTimeStamp(new TimeStamp(() -> { //Intake for 3 seconds
            intake.intakeForDuration(3);
        }, 0, 9));


        automous.addPath(-10, 33, toScore4, 0, 3); //10 - Score fourth sample

        automous.addLiftTimeStamp(new LiftTimeStamp(-10, 33, 1, 10)); //Put arm in scoring position


        automous.addPath(5, 5, toBar, 0, 4); //11 - Move to bar

        automous.addLiftTimeStamp(new LiftTimeStamp(5, 5, 2, 11)); //Touch bar with arm
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
        follower.update();
        leftLift.update();
        leftPivot.update();
        rightLift.update();
        rightPivot.update();
        intake.update();

        telemetry.addData("path state", currentState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("deltaTime", deltaTime);
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
        follower.breakFollowing();
    }
}

/**
 * 8==D
 */