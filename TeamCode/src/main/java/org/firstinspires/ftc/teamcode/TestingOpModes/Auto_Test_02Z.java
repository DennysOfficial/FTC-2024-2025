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
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ClawAndStuff;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.List;

@Autonomous(name = "Spare: SoupcOpMode_0-2-Z 1.0.8", group = "SoupcOpModes")
public class Auto_Test_02Z extends OpMode{

    List<LynxModule> allHubs;

    private final Pose startPose = new Pose(9,64.5, Math.toRadians(0));  // This is where the robot starts

    //Points of Interest
    Point rungPoint1 = new Point(35, 64.5, Point.CARTESIAN);
    Point rungPoint2 = new Point(35, 67.5, Point.CARTESIAN);

    Point rungPointControl1 = new Point(20, 28, Point.CARTESIAN);
    Point rungPointControl2 = new Point(20, 66, Point.CARTESIAN);

    Point pickupPoint2 = new Point(12, 28, Point.CARTESIAN);
    Point pickupPoint3 = new Point(11, 28, Point.CARTESIAN);

    public PathChain movement1, movement2, movement3;

    // Other misc. stuff
    private Follower follower;

    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;

    LeftLift leftLift;
    LeftPivot leftPivot;
    ActiveIntakeMotor intake;

    RightLift rightLift;
    RightPivot rightPivot;
    ClawAndStuff grabber;

    RobotConfig config;

    private Timer pathTimer;

    int listPointer = 0;

    @Override
    public void init() {

        pathTimer = new Timer();

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
        grabber = new ClawAndStuff(this, config, leftLift, leftPivot);

        leftLift.assignPivot(leftPivot);
        leftPivot.assignLift(leftLift);

        rightLift.assignPivot(rightPivot);
        rightPivot.assignLift(rightLift);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        rightLift.setTargetPosition(0);
        rightPivot.setTargetPosition(-60);
        grabber.Collect();
        leftLift.setTargetPosition(0);
        leftPivot.setTargetPosition(-50);
        grabber.closeClaw();
    }

    @Override
    public void init_loop() {
        leftLift.update();
        leftPivot.update();
        rightLift.update();
        rightPivot.update();
    }



    public void buildPaths() {
        movement1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(startPose), rungPoint1)))
                .build();
        movement2 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(rungPoint1, rungPointControl2, rungPointControl1, pickupPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0, () -> {
                    grabber.Collect();
                    grabber.openClaw();
                })
                .setZeroPowerAccelerationMultiplier(2)
                .addPath(new Path(new BezierLine(pickupPoint2, pickupPoint3)))
                .setZeroPowerAccelerationMultiplier(2)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        movement3 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(pickupPoint3, rungPointControl1, rungPointControl2, rungPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0, () -> {
                    grabber.closeClaw();
                })
                .addParametricCallback(0.1, () -> {
                    grabber.Score();
                })
                .addPath(new Path(new BezierCurve(rungPoint2, rungPointControl2, rungPointControl1, pickupPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0, () -> {
                    grabber.Collect();
                })
                .build();
    }

    public void routine() {
        switch (listPointer) {
            case 0:
                if (leftPivot.getPosition() >= 15) {
                    follower.followPath(movement1);
                    listPointer = 1;
                    pathTimer.resetTimer();
                }
                break;
            case 1:
                if (follower.atParametricEnd()) {
                    grabber.Collect();
                    if (leftPivot.getPosition() >= -25) {
                        follower.followPath(movement2);
                        listPointer = 2;
                    }
                }
                break;
            case 2:
                if (follower.atParametricEnd()) {
                    listPointer = 3;
                    pathTimer.resetTimer();
                }
                break;
            case 3:
                grabber.closeClaw();
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    grabber.Score();
                    if (leftPivot.getPosition() >= 15) {
                        follower.followPath(movement3);
                        listPointer = 4;
                    }
                }
                break;
            case 4:
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

        routine();
        follower.update();
        leftLift.update();
        leftPivot.update();
        rightLift.update();
        rightPivot.update();
        intake.update();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("deltaTime", deltaTime);
        telemetry.addData("waitTime", time);
        telemetry.addData("pathtimer", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    @Override
    public void start() {
        buildPaths();
        grabber.Score();
        pathTimer.resetTimer();

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