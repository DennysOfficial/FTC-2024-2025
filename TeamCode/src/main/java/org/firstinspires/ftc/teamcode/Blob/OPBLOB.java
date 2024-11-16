package org.firstinspires.ftc.teamcode.Blob;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.ArrayList;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class OPBLOB extends LinearOpMode {
    public void runOpMode() {

        String PixelColor = "Blue";
        double XCameraResolutionHeight = 640;
        double YCameraResolutionWidth = 480;
        double CameraAngle = 45;
        ArrayList<Double> Camera = new ArrayList<Double>();
        Camera.add(0,XCameraResolutionHeight);
        Camera.add(1,YCameraResolutionWidth);
        Camera.add(3,CameraAngle);
        Blob NewBlob = new Blob();
        ColorBlobLocatorProcessor colorLocator = NewBlob.CameraSetUp(PixelColor, (int) XCameraResolutionHeight, (int) YCameraResolutionWidth);

        SparkFunOTOS.Pose2D SampleCenter = new SparkFunOTOS.Pose2D();

        Vector3D VectorToCam = new Vector3D(0,0,0);

        SparkFunOTOS.Pose2D RobotPose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D SamplePose = new SparkFunOTOS.Pose2D();
        ArrayList<Double> CameraOffsets = new ArrayList<Double>();
        //loop all bellow


        NewBlob.GetSampleCenter(colorLocator, SampleCenter);

        //ArrayList<Double> VectorToRobot = new ArrayList<Double>();
        //NewBlob.VectorToRobot(VectorToRobot, RobotX,  RobotY, RobotHeading);

        NewBlob.CameraOffsetSetup(CameraOffsets);

        NewBlob.CamOffsetVectorFromOrgin(CameraOffsets, SamplePose, VectorToCam);

        NewBlob.SampleLocation(SampleCenter, VectorToCam, Camera ,SamplePose, RobotPose);

        telemetry.addData("x",SamplePose.x);
        telemetry.addData("y",SamplePose.y);



    }
}
