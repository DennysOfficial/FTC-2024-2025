package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SampleMecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg
    public static  double ANGLE2 = 0;
    public static int REPEAT = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        while (REPEAT < 10) {
            drive.turn(Math.toRadians(ANGLE));
            REPEAT = REPEAT + 1;
        }

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

    }
}
