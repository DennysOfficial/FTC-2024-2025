package org.firstinspires.ftc.teamcode.Blob;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.ArrayList;
import java.util.Vector;

public class OPBLOB extends LinearOpMode {
    public void runOpMode() {

        String PixelColor = "Blue";
        int XCameraResolutionHeight = 640;
        int YCameraResolutionWidth = 480;
        Blob NewBlob = new Blob();
        ColorBlobLocatorProcessor colorLocator = NewBlob.CameraSetUp(PixelColor, XCameraResolutionHeight, YCameraResolutionWidth);
        ArrayList<Double> SampleCenter = new ArrayList<Double>();


        NewBlob.GetSampleCenter(colorLocator, SampleCenter);
        double RobotX = 0;
        double RobotY = 0;
        double RobotHeading = 0;
        //ArrayList<Double> VectorToRobot = new ArrayList<Double>();
        //NewBlob.VectorToRobot(VectorToRobot, RobotX,  RobotY, RobotHeading);

        SparkFunOTOS.Pose2D RobotPose = new SparkFunOTOS.Pose2D();
        ArrayList<Double> CameraOffsets = new ArrayList<Double>();

        Vector VectorToCam = new Vector;
        NewBlob.CameraOffsetSetup(CameraOffsets);

        NewBlob.CamOffsetVectorFromOrgin(CameraOffsets, RobotPose, VectorToCam);


    }
}
