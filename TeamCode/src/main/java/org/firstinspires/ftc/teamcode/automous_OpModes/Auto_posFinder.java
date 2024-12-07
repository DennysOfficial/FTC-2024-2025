package org.firstinspires.ftc.teamcode.automous_OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Lift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Pivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveIntake;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ReadOnlyRuntime;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.List;

@Autonomous(name = "SoupcOpMode Position Finder")
public class Auto_posFinder extends OpMode{

    // TODO: Put paths here


    List<LynxModule> allHubs;

    double posX = 9;
    double posY = 72;
    double headingDeg = 0;

    double liftPos = 0;
    double pivotPos = 0;

    private final Pose startPose = new Pose(9,72, Math.toRadians(0));  // This is where the robot starts

    //Points of Interest



    // TODO: List paths the robot takes
    //public Path;
    //public PathChain;

    // Other misc. stuff
    private Follower follower;

    private final ReadOnlyRuntime runtime = new ReadOnlyRuntime();
    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;

    Lift lift;
    Pivot spinyBit;
    ActiveIntake intake;
    RobotConfig config;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk


        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        config = new RobotConfig(this);

        lift = new Lift(ControlAxis.ControlMode.positionControl,this, config);
        spinyBit = new Pivot(ControlAxis.ControlMode.positionControl,this, config);
        intake = new ActiveIntake(this, config);

        lift.assignPivot(spinyBit);
        spinyBit.assignLift(lift);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
    }



    public void buildPaths() {
        // TODO: Put path stuff here
    }

    public void autonomousPathUpdate() {
        // TODO: Put instructions in the switchcase
        follower.holdPoint(new BezierPoint(new Point(posX, posY, Point.CARTESIAN)), Math.toRadians(headingDeg));
        lift.setTargetPosition(liftPos);
        spinyBit.setTargetPosition(pivotPos);
    }

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        deltaTime = frameTimer.seconds();
        frameTimer.reset();

        intake.closeFlap();

        follower.update();
        lift.update();
        spinyBit.update();
        intake.update();

        autonomousPathUpdate();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("deltaTime", deltaTime);
        telemetry.addData("runTime", runtime);
        telemetry.addData("waitTime", time);
        telemetry.update();
    }



    @Override
    public void start() {
        buildPaths();
        deltaTime = 0;
        frameTimer.reset();
        // Put first path instead of brackets
        //currentState = State.[];
        //follower.followPath([]);
    }

    @Override
    public void stop() {
    }
}

/**
 * 8==D
 */