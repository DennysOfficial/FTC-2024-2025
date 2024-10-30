package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Automous extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(new Vector2d(72,0), Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        Trajectory toRung = drive.trajectoryBuilder(new Pose2d(72, 0, 180))
                .splineTo(new Vector2d(36,0), Math.toRadians(180))
                .build();

        Trajectory toSamples = drive.trajectoryBuilder(toRung.end())
                .strafeLeft(48.5)
                .build();

        Trajectory toNet = drive.trajectoryBuilder(toSamples.end())
                .lineToLinearHeading(new Pose2d(54, -54, Math.toRadians(135)))
                .build();
        Trajectory toAscentZone = drive.trajectoryBuilder(toNet.end())
                .splineTo(new Vector2d(12, -24), Math.toRadians(90))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(toRung);
        //[place] SPECIMEN on RUNG
        drive.followTrajectory(toSamples);
        //[grab] SAMPLE
        drive.followTrajectory(toNet);
        //[place] SAMPLE into HIGHNET
        drive.followTrajectory(toAscentZone);
        //Touch LOWRUNG with [grabber]

    }
}