package org.firstinspires.ftc.teamcode.Autonomous.blob;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

public class OPBLOB extends LinearOpMode {
    public void runOpMode() {

        String PixelColor = "Blue";
        int XCameraResolutionHeight = 640;
        int YCameraResolutionWidth = 480;
        Blob NewBlob = new Blob();
        ColorBlobLocatorProcessor colorLocator = NewBlob.CameraSetUp(PixelColor, XCameraResolutionHeight, YCameraResolutionWidth);
        double SampleXCenterCamera = 0;
        double SampleYCenterCamera = 0;

        NewBlob.GetSampleCenter(colorLocator, SampleXCenterCamera, SampleYCenterCamera);

    }
}
