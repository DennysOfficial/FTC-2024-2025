package org.firstinspires.ftc.teamcode.Blob;

import com.acmerobotics.roadrunner.Pose2d;
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
    double positionTolerance = 0.05;

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

    public void difference(){
        // If the list is empty or has only one element, no duplicates possible
        if (deepCopy == null || deepCopy.size() <= 1) {
            return;
        }

        // Compare each pose with every other pose
        for (int i = 0; i < deepCopy.size(); i++) {
            for (int j = i + 1; j < deepCopy.size(); j++) {
                Pose2D pose1 = deepCopy.get(i);
                Pose2D pose2 = deepCopy.get(j);

                // Calculate position difference
                double xDiff = pose1.getX(DistanceUnit.INCH) - pose2.getX(DistanceUnit.INCH);
                double yDiff = pose1.getY(DistanceUnit.INCH) - pose2.getY(DistanceUnit.INCH);
                double posDiff = Math.sqrt(xDiff * xDiff + yDiff * yDiff);

                // If positions are within tolerance, remove the duplicate
                if (posDiff <= positionTolerance) {
                    deepCopy.remove(j);
                    j--; // Adjust index since we removed an element
                }
            }
        }
    }
}