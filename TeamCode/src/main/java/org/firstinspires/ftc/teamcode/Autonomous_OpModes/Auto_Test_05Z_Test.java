package org.firstinspires.ftc.teamcode.Autonomous_OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.SpecimenArm;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.RightLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.RightPivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveSpecimenClaw;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "OtherOpModetoRuleThemAll")
public class Auto_Test_05Z_Test extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    RobotConfig config;

    RightLift rightLift;
    RightPivot rightPivot;
    ActiveSpecimenClaw grabber;

    SpecimenArm spArm;
    Servo wristServo;

    private final Pose startPose = new Pose(9.25,48.5, Math.toRadians(0));  // This is where the robot starts

    Point rungPoint3 = new Point(34, 67.5, Point.CARTESIAN);
    Point rungPoint4 = new Point(30, 66, Point.CARTESIAN);
    Point rungPoint5 = new Point(30, 64.5, Point.CARTESIAN);
    Point rungPoint2 = new Point(30, 69, Point.CARTESIAN);
    Point rungPoint1 = new Point(30, 70.5, Point.CARTESIAN);
    Point rungPoint3a = new Point(22, 67.5, Point.CARTESIAN);
    Point rungPoint4a = new Point(22, 66, Point.CARTESIAN);
    Point rungPoint5a = new Point(22, 64.5, Point.CARTESIAN);
    Point rungPoint2a = new Point(22, 69, Point.CARTESIAN);
    Point rungPoint1a = new Point(22, 70.5, Point.CARTESIAN);

    Point rungPointControl1 = new Point(20,28, Point.CARTESIAN);
    Point rungPointControl2 = new Point(20, 66, Point.CARTESIAN);

    Point samplecurvepoint1 = new Point(17,20, Point.CARTESIAN);
    Point samplecurvepoint2 = new Point(72,48, Point.CARTESIAN);
    Point samplecurvepoint3 = new Point(72,30, Point.CARTESIAN);
    Point samplecurvepoint4 = new Point(72,20, Point.CARTESIAN);

    Point samplepoint1 =      new Point(60,26.5, Point.CARTESIAN);
    Point samplepoint2 =      new Point(60,17, Point.CARTESIAN);
    Point samplepoint3 =      new Point(60,12.5, Point.CARTESIAN);

    Point linepoint1 =        new Point(32,25, Point.CARTESIAN);
    Point linepoint2 =        new Point(32,17, Point.CARTESIAN);
    Point linepoint3 =        new Point(32,12.5, Point.CARTESIAN);

    Point pickupPoint2 = new Point(14.5, 26, Point.CARTESIAN);
    Point pickupPoint3 = new Point(10.5, 26, Point.CARTESIAN);

    public Path toSample1, toSample2, toSample3, toline1, toline2, toline3, score1a, collect2;

    public PathChain score1, score2, collect3, score3, moveSamples, collect4, score4, collect5, score5, score2a, score3a, score4a, score5a;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        score1 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(startPose), rungPointControl2, rungPoint1a)))
                .setPathEndTimeoutConstraint(100)
                .setPathEndVelocityConstraint(0.15)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        toSample1 = new Path(new BezierCurve(rungPoint1, samplecurvepoint1, samplecurvepoint2, samplepoint1));
        toSample2 = new Path(new BezierCurve(linepoint1, samplecurvepoint3, samplepoint2));
        toSample3 = new Path(new BezierCurve(linepoint2, samplecurvepoint4, samplepoint3));

        toline1 = new Path(new BezierLine(samplepoint1, linepoint1));
        toline2 = new Path(new BezierLine(samplepoint2, linepoint2));
        toline3 = new Path(new BezierLine(samplepoint3, linepoint3));

        collect2 = new Path(new BezierLine(linepoint3, pickupPoint3));

        score1a = new Path(new BezierLine(rungPoint1a, rungPoint1));

        moveSamples = follower.pathBuilder()
                .addPath(score1a)
                .addPath(toSample1)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0, () -> {
                    spArm.armState = SpecimenArm.SpecimenArmState.collect;
                    grabber.openClaw();
                })
                .addPath(toline1)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(toSample2)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(toline2)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(toSample3)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(toline3)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(collect2)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        score2 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(pickupPoint3, rungPointControl1, rungPointControl2, rungPoint2a)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(50)
                .setPathEndVelocityConstraint(0.3)
                .addPath(new Path(new BezierLine(rungPoint2a, rungPoint2)))
                .addPath(new Path(new BezierCurve(rungPoint2, pickupPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(100)
                .setPathEndVelocityConstraint(0.15)
                .addParametricCallback(0, () -> {
                    spArm.armState = SpecimenArm.SpecimenArmState.collect;
                    grabber.openClaw();
                })
                .build();

        collect3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickupPoint2, pickupPoint3)))
                .setZeroPowerAccelerationMultiplier(5)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(350)
                .build();
        score3 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(pickupPoint3, rungPointControl1, rungPointControl2, rungPoint3a)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(50)
                .setPathEndVelocityConstraint(0.3)
                .addPath(new Path(new BezierLine(rungPoint3a, rungPoint3)))
                .addPath(new Path(new BezierCurve(rungPoint3, pickupPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(100)
                .setPathEndVelocityConstraint(0.15)
                .addParametricCallback(0, () -> {
                    spArm.armState = SpecimenArm.SpecimenArmState.collect;
                    grabber.openClaw();
                })
                .build();

        collect4 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickupPoint2, pickupPoint3)))
                .setZeroPowerAccelerationMultiplier(5)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(350)
                .build();

        score4 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(pickupPoint3, rungPointControl1, rungPointControl2, rungPoint4a)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(50)
                .setPathEndVelocityConstraint(0.3)
                .addPath(new Path(new BezierLine(rungPoint4a, rungPoint4)))
                .addPath(new Path(new BezierCurve(rungPoint4, pickupPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(100)
                .setPathEndVelocityConstraint(0.15)
                .addParametricCallback(0, () -> {
                    spArm.armState = SpecimenArm.SpecimenArmState.collect;
                    grabber.openClaw();
                })
                .build();

        collect5 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickupPoint2, pickupPoint3)))
                .setZeroPowerAccelerationMultiplier(5)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(350)
                .build();

        score5 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(pickupPoint3, rungPointControl1, rungPointControl2, rungPoint5a)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(50)
                .setPathEndVelocityConstraint(0.3)
                .addPath(new Path(new BezierLine(rungPoint5a, rungPoint5)))
                .addPath(new Path(new BezierCurve(rungPoint4, pickupPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0, () -> {
                    spArm.armState = SpecimenArm.SpecimenArmState.collect;
                    grabber.openClaw();
                })
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    setPathState(1);
                    follower.followPath(score1);
                }
                break;
            case 1:
                if (!follower.isBusy() && !spArm.isBusy()) {
                    follower.followPath(moveSamples);
                    if (follower.isBusy()) setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    setPathState(3);
                    grabber.closeClawHard();
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() >= 0.4) {
                    spArm.armState = SpecimenArm.SpecimenArmState.movingToScore;
                    follower.followPath(score2);
                    if (follower.isBusy()) setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    if (follower.isBusy()) setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    grabber.openClaw();
                    follower.followPath(collect3);
                    if (follower.isBusy()) setPathState(6);

                }
            case 6:
                if (!follower.isBusy()) {
                    setPathState(7);
                    grabber.closeClawHard();
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() >= 0.4) {
                    spArm.armState = SpecimenArm.SpecimenArmState.movingToScore;
                    follower.followPath(score3);
                    if (follower.isBusy()) setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    if (follower.isBusy()) setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    grabber.openClaw();
                    follower.followPath(collect4);
                    if (follower.isBusy()) setPathState(10);

                }
            case 10:
                if (!follower.isBusy()) {
                    setPathState(11);
                    grabber.closeClawHard();
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() >= 0.4) {
                    spArm.armState = SpecimenArm.SpecimenArmState.movingToScore;
                    follower.followPath(score4);
                    if (follower.isBusy()) setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    if (follower.isBusy()) setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    grabber.openClaw();
                    follower.followPath(collect5);
                    if (follower.isBusy()) setPathState(14);
                }
            case 14:
                if (!follower.isBusy()) {
                    setPathState(16);
                    grabber.closeClawHard();
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() >= 0.4) {
                    spArm.armState = SpecimenArm.SpecimenArmState.movingToScore;
                    follower.followPath(score5);
                    if (follower.isBusy()) setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    if (follower.isBusy()) setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        rightLift.update();
        rightPivot.update();
        spArm.autoUpdate();

        // Feedback to Driver Hub
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("time left", 30 - opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("waitTime", time);
        telemetry.addData("pathtimer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Path State", pathState);
        telemetry.addData("busy", follower.isBusy());
        telemetry.update();

        if (opmodeTimer.getElapsedTimeSeconds() >= 30) stop();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        config = new RobotConfig(this);

        rightLift = new RightLift(ControlAxis.ControlMode.positionControl,this, config);
        rightPivot = new RightPivot(ControlAxis.ControlMode.positionControl,this, config);
        grabber = new ActiveSpecimenClaw(this, config);

        spArm = new SpecimenArm(this, config);

        rightLift.assignPivot(rightPivot);
        rightPivot.assignLift(rightLift);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        rightLift.setTargetPosition(0);
        rightPivot.setTargetPosition(-50);
        spArm.armState = SpecimenArm.SpecimenArmState.rest;
        grabber.closeClawHard();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        rightLift.update();
        rightPivot.update();
        spArm.autoUpdate();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        spArm.armState = SpecimenArm.SpecimenArmState.movingToScore;
        follower.followPath(score1);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        follower.breakFollowing();
        spArm.armState = SpecimenArm.SpecimenArmState.collect;
    }
}

