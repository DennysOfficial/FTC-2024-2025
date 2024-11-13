package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutonomouseStuff.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes.BasicMechanumDrive;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes.DriveModeBase;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Lift;
import org.firstinspires.ftc.teamcode.AutonomouseStuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Pivot;


@Autonomous(name = "please dont break robot", group = "Autonomous")
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk


        Pose2d initialPose = new Pose2d(0, 72-18f/2, Math.toRadians(90));

        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);

        RobotConfig activeConfig = drive.config; // selects the active setting that will be used in the rest of the code


        Lift lift = new Lift(this, activeConfig, runtime);

        lift.setControlMode(ControlAxis.ControlMode.positionControl);


        Pivot spinnyBit = new Pivot(this, activeConfig, runtime);

        spinnyBit.setControlMode(ControlAxis.ControlMode.positionControl);


        //TrajectoryActionBuilder driveIntoDaBar = drive.actionBuilder(initialPose).

        TrajectoryActionBuilder driveIntoDaBar = drive.actionBuilder(initialPose)
                .lineToY(-30);



        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(claw.closeClaw());


        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        SequentialAction actualAutonomousStuff = new SequentialAction(
                spinnyBit.actionGoToPosition(0),
                lift.actionGoToPosition(9),
                driveIntoDaBar.build()
        );

        ParallelAction mainLoop = new ParallelAction(
                actualAutonomousStuff,
                spinnyBit.actionUpdate(),
                lift.actionUpdate()
        );

        Actions.runBlocking(
                mainLoop
        );
    }
}