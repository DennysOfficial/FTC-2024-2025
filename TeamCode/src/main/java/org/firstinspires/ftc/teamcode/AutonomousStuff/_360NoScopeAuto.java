package org.firstinspires.ftc.teamcode.AutonomousStuff;

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

// IMPORTANT: Setup
//Start: right edge 38.5 from wall
//2nd pickup: 30.25 from wall
//3rd-5th pickup: 28 from wall

@Autonomous(name = "if don't work actually brain dead fr fr no cap skull emoji 5x")
//@Disabled
public class _360NoScopeAuto extends OpMode {

    private Follower follower;
    private Timer stateTimer;

    /**
     * This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.
     */
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

    private final Pose startPose = new Pose(9.25, 48.5, Math.toRadians(0));  // This is where the robot starts

    Point rungPoint3 = EpicPoints.ab_rungPoint3.convertToPoint();
    Point rungPoint4 = EpicPoints.ad_rungPoint4.convertToPoint();
    Point rungPoint2 = EpicPoints.ac_rungPoint2.convertToPoint();
    Point rungPoint1 = EpicPoints.aa_rungPoint1.convertToPoint();

    Point rungPoint3Approach = EpicPoints.bc_rungApproachPoint3.convertToPoint();
    Point rungPoint4Approach = EpicPoints.bd_rungApproachPoint4.convertToPoint();
    Point rungPoint2Approach = EpicPoints.bb_rungApproachPoint2.convertToPoint();
    Point rungPoint1Approach = EpicPoints.ba_rungApproachPoint1.convertToPoint();

    Point scorePathControlPoint1 = EpicPoints.ca_scorePathControlPoint1.convertToPoint();
    Point scorePathControlPoint2 = EpicPoints.cb_scorePathControlPoint2.convertToPoint();

    Point sampleCurvePoint1 = EpicPoints.sampleCurvePoint1.convertToPoint();
    Point sampleCurvePoint2 = EpicPoints.sampleCurvePoint2.convertToPoint();
    Point sampleCurvePoint3 = EpicPoints.sampleCurvePoint3.convertToPoint();
    Point sampleCurvePoint4 = EpicPoints.sampleCurvePoint4.convertToPoint();

    Point samplepoint1 = new Point(54, 26.5, Point.CARTESIAN);
    Point samplepoint2 = new Point(54, 17.5, Point.CARTESIAN);
    Point samplepoint3 = new Point(56, 12, Point.CARTESIAN);

    Point linepoint1 = new Point(30, 25, Point.CARTESIAN);
    Point linepoint2 = new Point(30, 17.7, Point.CARTESIAN);
    Point linepoint3 = new Point(28, 11.85, Point.CARTESIAN);

    Point pickupPointApproach = EpicPoints.da_pickupPointApproach.convertToPoint();
    Point pickupPointAtWallLastFew = EpicPoints.db_pickupPointAtWallDefault.convertToPoint();
    Point pickupPointAtWallFirst = EpicPoints.dc_pickupPointFirstWallPickup.convertToPoint();

    public Path toSample1, toSample2, toSample3, toline1, toline2, toline3, score1a, collect2, collect2a;

    public PathChain score1, score2, collect3, score3, moveSamples, collect4, score4, collect5, score5, score2a, score3a, score4a;

    /**
     * Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts.
     **/
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
                .addPath(new Path(new BezierCurve(new Point(startPose), scorePathControlPoint2, rungPoint1Approach)))
                .setPathEndTimeoutConstraint(100)
                .setPathEndVelocityConstraint(0.15)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        toSample1 = new Path(new BezierCurve(rungPoint1, sampleCurvePoint1, sampleCurvePoint2, samplepoint1));
        toSample2 = new Path(new BezierCurve(linepoint1, sampleCurvePoint3, samplepoint2));
        toSample3 = new Path(new BezierCurve(linepoint2, sampleCurvePoint4, samplepoint3));

        toline1 = new Path(new BezierLine(samplepoint1, linepoint1));
        toline2 = new Path(new BezierLine(samplepoint2, linepoint2));
        toline3 = new Path(new BezierLine(samplepoint3, linepoint3));

        collect2 = new Path(new BezierLine(linepoint3, pickupPointApproach));
        collect2a = new Path(new BezierLine(pickupPointApproach, pickupPointAtWallFirst));

        score1a = new Path(new BezierLine(rungPoint1Approach, rungPoint1));

        score2a = follower.pathBuilder()
                .addPath(new Path(new BezierLine(rungPoint2Approach, rungPoint2)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new Path(new BezierCurve(rungPoint2, pickupPointApproach)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(100)
                .setPathEndVelocityConstraint(0.15)
                .addParametricCallback(0, () -> {
                    spArm.armState = SpecimenArm.SpecimenArmState.collect;
                    grabber.openClaw();
                })
                .addParametricCallback(0.1, () -> {
                    grabber.openClaw();
                })
                .addParametricCallback(0.95, () -> grabber.closeClawHard())
                .build();
        score3a = follower.pathBuilder()
                .addPath(new Path(new BezierLine(rungPoint3Approach, rungPoint3)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new Path(new BezierCurve(rungPoint3, pickupPointApproach)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(100)
                .setPathEndVelocityConstraint(0.15)
                .addParametricCallback(0, () -> {
                    spArm.armState = SpecimenArm.SpecimenArmState.collect;
                    grabber.openClaw();
                })
                .addParametricCallback(0.1, () -> {
                    grabber.openClaw();
                })
                .addParametricCallback(0.95, () -> grabber.closeClawHard())
                .build();
        score4a = follower.pathBuilder()
                .addPath(new Path(new BezierLine(rungPoint4Approach, rungPoint4)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new Path(new BezierCurve(rungPoint4, rungPoint4Approach, pickupPointApproach)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(100)
                .setPathEndVelocityConstraint(0.15)
                .addParametricCallback(0, () -> {
                    spArm.armState = SpecimenArm.SpecimenArmState.collect;
                    grabber.openClaw();
                })
                .addParametricCallback(0.1, () -> {
                    grabber.openClaw();
                })
                .addParametricCallback(0.95, () -> grabber.closeClawHard())
                .build();

        moveSamples = follower.pathBuilder()
                .addPath(score1a)
                .setConstantHeadingInterpolation(Math.toRadians(0))
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
                .setPathEndTimeoutConstraint(100)
                .setPathEndVelocityConstraint(0.15)
                .addParametricCallback(0.95, () -> grabber.closeClawHard())
                .addPath(collect2a)
                .setZeroPowerAccelerationMultiplier(2.5)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(350)
                .build();

        score2 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(pickupPointAtWallLastFew, scorePathControlPoint1, scorePathControlPoint2, rungPoint2Approach)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(50)
                .setPathEndVelocityConstraint(1)
                .build();

        collect3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickupPointApproach, pickupPointAtWallLastFew)))
                .setZeroPowerAccelerationMultiplier(2.5)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(350)
                .build();
        score3 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(pickupPointAtWallLastFew, scorePathControlPoint1, scorePathControlPoint2, rungPoint3Approach)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(50)
                .setPathEndVelocityConstraint(1)
                .build();

        collect4 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickupPointApproach, pickupPointAtWallLastFew)))
                .setZeroPowerAccelerationMultiplier(2.5)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(350)
                .build();

        score4 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(pickupPointAtWallLastFew, scorePathControlPoint1, scorePathControlPoint2, rungPoint4Approach)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(50)
                .setPathEndVelocityConstraint(1)
                .build();

        collect5 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickupPointApproach, pickupPointAtWallLastFew)))
                .setZeroPowerAccelerationMultiplier(2.5)
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndTimeoutConstraint(350)
                .build();
    }

    /**
     * This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on.
     */
    public void autonomousPathUpdate() {
        telemetry.addData("path state", pathState);
        switch (pathState) {
            case 0:
                follower.followPath(score1);
                setPathStateAndResetTimer(213);
            case 213:
                if (follower.isBusy())
                    setPathStateAndResetTimer(124);

            case 124:
                if (!follower.isBusy())
                    setPathStateAndResetTimer(43523);

            case 43523:
                if (stateTimer.getElapsedTimeSeconds() < EpicPoints.AAA_initialArmSettleWait)
                    return;
                setPathStateAndResetTimer(1);
                break;
            case 1:
                if (follower.isBusy() || spArm.isBusy())
                    return;
                follower.followPath(moveSamples);
                setPathStateAndResetTimer(2);
            case 2:
                if (follower.isBusy())
                    setPathStateAndResetTimer(3);
                break;

            case 3:
                if (follower.isBusy())
                    return;
                spArm.armState = SpecimenArm.SpecimenArmState.movingToScore;
                follower.followPath(score2);
                setPathStateAndResetTimer(420);
            case 420:
                if (follower.isBusy()) setPathStateAndResetTimer(4);
                break;

            case 4:
                if (follower.isBusy())
                    return;
                follower.followPath(score2a);
                setPathStateAndResetTimer(69);

            case 69:
                if (follower.isBusy()) setPathStateAndResetTimer(5);
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(collect3);
                    setPathStateAndResetTimer(6);
                }
            case 6:
                if (follower.isBusy()) setPathStateAndResetTimer(7);
                break;

            case 7:
                if (follower.isBusy())
                    return;
                spArm.armState = SpecimenArm.SpecimenArmState.movingToScore;
                follower.followPath(score3);
                setPathStateAndResetTimer(123);
            case 123:
                if (follower.isBusy()) setPathStateAndResetTimer(8);
                break;

            case 8:
                if (follower.isBusy())
                    return;
                follower.followPath(score3a);
                setPathStateAndResetTimer(42069);
            case 42069:
                if (follower.isBusy()) setPathStateAndResetTimer(9);
                break;

            case 9:
                if (follower.isBusy())
                    return;
                follower.followPath(collect4);
                setPathStateAndResetTimer(10);
            case 10:
                if (follower.isBusy()) setPathStateAndResetTimer(11);
                break;

            case 11:
                if (follower.isBusy())
                    return;
                spArm.armState = SpecimenArm.SpecimenArmState.movingToScore;
                follower.followPath(score4);
                setPathStateAndResetTimer(635);
                break;

            case 635:
                if (follower.isBusy()) setPathStateAndResetTimer(12);
                break;

            case 12:
                if (follower.isBusy())
                    return;
                follower.followPath(score4a);
                setPathStateAndResetTimer(2354);

            case 2354:
                if (follower.isBusy()) setPathStateAndResetTimer(13);
                break;
            case 13:
                if (follower.isBusy())
                    return;
                follower.followPath(collect5);
                setPathStateAndResetTimer(14);

            case 14:
                if (follower.isBusy()) setPathStateAndResetTimer(16);
                break;

            case 16:
                if (follower.isBusy())
                    return;
                spArm.armState = SpecimenArm.SpecimenArmState.movingToScore;
                follower.followPath(score4);
                setPathStateAndResetTimer(69420);

            case 69420:
                if (follower.isBusy()) setPathStateAndResetTimer(17);
                break;

            case 17:
                if (!follower.isBusy())
                    return;
                follower.followPath(score4a);
                setPathStateAndResetTimer(2343);
            case 2343:
                if (follower.isBusy()) setPathStateAndResetTimer(5318008);
                break;

            default:
                telemetry.addLine("fini");
        }
    }

    /**
     * These change the states of the paths and actions
     * It will also reset the timers of the individual switches
     **/
    public void setPathStateAndResetTimer(int pState) {
        pathState = pState;
        stateTimer.resetTimer();
    }


    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
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
        telemetry.addData("waitTime", time);
        telemetry.addData("pathtimer", stateTimer.getElapsedTimeSeconds());
        telemetry.addData("Path State", pathState);
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("parametric time", follower.getCurrentTValue());
        telemetry.update();

        follower.drawOnDashBoard();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk

        stateTimer = new Timer();

        config = new RobotConfig(this);

        rightLift = new RightLift(ControlAxis.ControlMode.positionControl, this, config);
        rightPivot = new RightPivot(ControlAxis.ControlMode.positionControl, this, config);
        grabber = new ActiveSpecimenClaw(this, config);

        spArm = new SpecimenArm(this, config);

        rightLift.assignPivot(rightPivot);
        rightPivot.assignLift(rightLift);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        rightLift.setTargetPosition(0);
        rightPivot.setTargetPosition(-55);
        spArm.armState = SpecimenArm.SpecimenArmState.rest;
        grabber.closeClawHard();
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
        rightLift.update();
        rightPivot.update();
        spArm.autoUpdate();
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        setPathStateAndResetTimer(0);
        spArm.armState = SpecimenArm.SpecimenArmState.movingToScore;
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}

