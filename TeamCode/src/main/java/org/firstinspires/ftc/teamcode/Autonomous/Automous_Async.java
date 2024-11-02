package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Config
@Autonomous(name = "soupcOpMode")
@Disabled
public class Automous_Async extends LinearOpMode {

    Trajectory toRung;
    Trajectory toSamples;
    Trajectory toNet;
    Trajectory toAscentZone;
    Pose2d startPose = new Pose2d(new Vector2d(72,0), Math.toRadians(180));

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this);

        drive.setPoseEstimate(startPose);

        toRung = drive.trajectoryBuilder(new Pose2d(72, 0, 180))
                .splineTo(new Vector2d(36,0), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    //[place] SPECIMEN on RUNG
                    drive.followTrajectoryAsync(toSamples);
                })
                .build();

        toSamples = drive.trajectoryBuilder(toRung.end())
                .strafeLeft(48.5)
                .addDisplacementMarker(() -> {
                    //[grab] SAMPLE
                    drive.followTrajectoryAsync(toNet);
                })
                .build();

        toNet = drive.trajectoryBuilder(toSamples.end())
                .lineToLinearHeading(new Pose2d(54, -54, Math.toRadians(135)))
                .addDisplacementMarker(() -> {
                    //[place] SAMPLE into HIGHNET
                    drive.followTrajectoryAsync(toAscentZone);
                })
                .build();

        toAscentZone = drive.trajectoryBuilder(toNet.end())
                .splineTo(new Vector2d(12, -24), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    //Touch LOWRUNG with [grabber]
                })
                .build();

        while (!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectoryAsync(toRung);

        while (!isStopRequested() && opModeIsActive()) {

            drive.update();

            // Put your PID Update Function Here
        }
    }
}