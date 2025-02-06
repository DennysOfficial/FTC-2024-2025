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
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.PassiveGrabber;
import org.firstinspires.ftc.teamcode.pedroPathing.Automous;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.List;

@Autonomous(name = "SoupcOpMode_0-2-X 0.0.1")
public class Auto_Test_02X extends OpMode{

    List<LynxModule> allHubs;

    private final Pose startPose = new Pose(9,64.5, Math.toRadians(0));  // This is where the robot starts

    //Points of Interest
    Point rungPoint1 = new Point(30, 64.5, Point.CARTESIAN);
    Point rungPoint2 = new Point(30, 67.5, Point.CARTESIAN);

    Point pickupPoint2 = new Point(12, 36, Point.CARTESIAN);
    Point pickupPoint3 = new Point(10.5, 36, Point.CARTESIAN);

    public PathChain movement1;

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
        rightPivot.setTargetPosition(-50);
        grabber.Collect();
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
                .addTemporalCallback(0.5, () -> {
                    grabber.Score();
                })
                .addPath(new Path(new BezierLine(rungPoint1, pickupPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addTemporalCallback(0.5, () -> {
                    grabber.Collect();
                    grabber.openClaw();
                })
                .addPath(new Path(new BezierLine(pickupPoint2, pickupPoint3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new Path(new BezierLine(pickupPoint3, rungPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addTemporalCallback(0, () -> {
                    grabber.closeClaw();
                })
                .addTemporalCallback(0.5, () -> {
                    grabber.Score();
                })
                .addPath(new Path(new BezierLine(rungPoint2, pickupPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addTemporalCallback(0.5, () -> {
                    grabber.Collect();
                })
                .build();
    }

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        deltaTime = frameTimer.seconds();
        frameTimer.reset();

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
        telemetry.update();
    }

    @Override
    public void start() {
        buildPaths();
        follower.followPath(movement1);
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