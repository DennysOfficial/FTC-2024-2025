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



    public BestSampleToPickUpAlgorithm(LinearOpMode opMode) {
        this.opMode = opMode;
    }


    public List<Pose2D> NewListOfSamples(List<Pose2D> allSampleGobalPositons){

        //List<Pose2D> deepCopy = new ArrayList<>();
        for (Pose2D pose : allSampleGobalPositons) {
            deepCopy.add(new Pose2D(DistanceUnit.INCH, pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), AngleUnit.RADIANS, pose.getHeading(AngleUnit.RADIANS)));
        }

        return deepCopy;
    }

    public void normToRobot(SparkFunOTOS.Pose2D robotPose){

         listLength = deepCopy.size();
         int i = 0;

         for (Pose2D pose : deepCopy){

             deepCopy.set(i, new Pose2D(DistanceUnit.INCH, pose.getX(DistanceUnit.INCH)- robotPose.x,pose.getY(DistanceUnit.INCH)- robotPose.y, AngleUnit.RADIANS, pose.getHeading(AngleUnit.RADIANS)));

             i += 1;
         }


    }


}
