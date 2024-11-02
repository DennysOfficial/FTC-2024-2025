package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;

@Config
@Autonomous(name = "soupcOpMode_Simple")
public class Automous_Async_Simple extends LinearOpMode {

    Trajectory toObservationZone;
    Pose2d startPose = new Pose2d(new Vector2d(60, 24), Math.toRadians(180));

    @Override
    public void runOpMode() {

        RobotConfig config = new RobotConfig(this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this);

        drive.setPoseEstimate(startPose);

        toObservationZone = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(60, 60))
                .build();

        while (!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectoryAsync(toObservationZone);

        while (!isStopRequested() && opModeIsActive()) {

            drive.update();

            // Put your PID Update Function Here
        }
    }
}