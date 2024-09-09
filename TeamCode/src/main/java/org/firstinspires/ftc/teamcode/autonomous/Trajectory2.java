package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Trajectory2 extends BasicMecanumOpMode {


    int propLocation;
    private final ElapsedTime elapsedTime = new ElapsedTime();
    int white = 0;
    int white1 = 0;
    double end = 30;
    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, -63, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        /*Improvements:
        fine tune the Coords
         */
        Trajectory moveToSpikes = drive.trajectoryBuilder(startPose)
                        .splineTo(new Vector2d(12, -36), Math.toRadians(propLocation * 90))
                        .build();
        Trajectory moveToBackground = drive.trajectoryBuilder(moveToSpikes.end())
                        .splineTo(new Vector2d(48, -36), Math.toRadians(0))
                        .lineToConstantHeading(new Vector2d(48, -42 + (propLocation * 6)))
                        .build();

        Trajectory moveToBackstage = drive.trajectoryBuilder(moveToBackground.end())
                .splineTo(new Vector2d(60, -60), Math.toRadians(0))
                .build();


        waitForStart();

        if(isStopRequested()) return;

        //Camera; Identify Team Prop location
        drive.followTrajectory(moveToSpikes);
        //Lift; Place Purple Pixel
        //liftMotorOne.setVelocity(0); //set to a low number
        //liftMotorTwo.setVelocity(0); //same
        //sleep(0); //change for far the lift needs to move
        //liftMotorOne.setVelocity(0); //don't change
        //liftMotorTwo.setVelocity(0); //don't change
        //sleep(100); //don't change
        //leftup.setPosition(0);
        //rightup.setPosition(0);
        //leftWrist.setPosition(0);
        //rightWrist.setPosition(0);
        //leftGrab.setPosition(0);
        //rightGrab.setPosition(0);


        drive.followTrajectory(moveToBackground);
        //Lift; Place Yellow Pixel
        //liftMotorOne.setVelocity(0); //set to a low number
        //liftMotorTwo.setVelocity(0);
        //sleep(0); //change for far the lift needs to move
        //liftMotorOne.setVelocity(0); //don't change
        //liftMotorTwo.setVelocity(0); //don't change
        //sleep(100); //don't change
        //leftup.setPosition(0);
        //rightup.setPosition(0);
        //leftWrist.setPosition(0);
        //rightWrist.setPosition(0);
        //leftGrab.setPosition(0);
        //rightGrab.setPosition(0);


        drive.followTrajectory(moveToBackstage);
        }




}





