package org.firstinspires.ftc.teamcode.Autonomous.blob;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.ArrayList;

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
        ArrayList<Double> VectorToRobot = new ArrayList<Double>();
        NewBlob.VectorToRobot(VectorToRobot, RobotX,  RobotY, RobotHeading);

        ArrayList<Double> CameraOffsets = new ArrayList<Double>();
        ArrayList<Double> VectorToCam = new ArrayList<Double>();
        NewBlob.CameraOffsetSetup(CameraOffsets);

        NewBlob.CamOffsetVectorFromOrgin(CameraOffsets, VectorToRobot, VectorToCam);


    }
}
