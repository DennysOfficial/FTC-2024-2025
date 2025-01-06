package org.firstinspires.ftc.teamcode.Blob;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;

public class BestSampleToPickUpAlgorithm {
    LinearOpMode opMode;

    List<Pose2D> deepCopy = new ArrayList<>();

    int listLength;

    double xDistanceFrom00toSub = 56.5;
    double yDistanceFrom00toSub = 48;


    public BestSampleToPickUpAlgorithm(LinearOpMode opMode) {
        this.opMode = opMode;
    }


    public void NewListOfSamples(List<Pose2D> allSampleGobalPositons){

        //List<Pose2D> deepCopy = new ArrayList<>();
        for (Pose2D pose : allSampleGobalPositons) {
            deepCopy.add(new Pose2D(DistanceUnit.INCH, pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), AngleUnit.RADIANS, pose.getHeading(AngleUnit.RADIANS)));
        }

    }

    public void normToSubCorner(SparkFunOTOS.Pose2D robotPose){

         listLength = deepCopy.size();
         int i = 0;

         for (Pose2D pose : deepCopy){

             deepCopy.set(i, new Pose2D(DistanceUnit.INCH, pose.getX(DistanceUnit.INCH)- xDistanceFrom00toSub,pose.getY(DistanceUnit.INCH)- yDistanceFrom00toSub, AngleUnit.RADIANS, pose.getHeading(AngleUnit.RADIANS)));

             i += 1;
         }

    }




}
