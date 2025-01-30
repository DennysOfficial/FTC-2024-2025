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
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.PassiveGrabber;
import org.firstinspires.ftc.teamcode.pedroPathing.AutomousNoLift;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.List;

@Autonomous(name = "SquareTest")
public class Auto_Test_Template extends OpMode{

    LeftLift leftLift;
    LeftPivot leftPivot;

    RightLift rightLift;
    RightPivot rightPivot;

    ActiveIntakeMotor intake;
    PassiveGrabber grabber;

    RobotConfig config;

    //AutomousNoLift automous;

    List<LynxModule> allHubs;

    // List of paths the robot takes
    private PathChain square;

    // Other misc. stuff
    private Follower follower;


    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;

    Pose startPose = new Pose(0, 0);

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

        //automous.addPath(0, 0, square, 0, 10);
    }

    public void autonomousPathUpdate() {

    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        deltaTime = frameTimer.seconds();
        frameTimer.reset();

        //automous.routine();
        follower.update();
        leftLift.update();
        leftPivot.update();
        rightLift.update();
        rightPivot.update();
        intake.update();

        //telemetry.addData("path", automous.getPath());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("deltatime", deltaTime);
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

        leftLift = new LeftLift(ControlAxis.ControlMode.positionControl,this, config);
        leftPivot = new LeftPivot(ControlAxis.ControlMode.positionControl,this, config);
        intake = new ActiveIntakeMotor(this, config);

        rightLift = new RightLift(ControlAxis.ControlMode.positionControl,this, config);
        rightPivot = new RightPivot(ControlAxis.ControlMode.positionControl,this, config);
        grabber = new PassiveGrabber(this, config, leftLift, leftPivot);

        leftLift.assignPivot(leftPivot);
        leftPivot.assignLift(leftLift);

        rightLift.assignPivot(rightPivot);
        rightPivot.assignLift(rightLift);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        //automous = new AutomousNoLift(this, config, follower);
    }

    @Override
    public void start() {
        buildPaths();
        deltaTime = 0;
        frameTimer.reset();
        follower.followPath(square);
    }

    @Override
    public void stop() {
    }
}

/**
 * 8==D
 */