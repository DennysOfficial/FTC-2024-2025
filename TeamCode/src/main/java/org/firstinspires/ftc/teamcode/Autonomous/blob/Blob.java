package org.firstinspires.ftc.teamcode.Autonomous.blob;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.ArrayList;
import java.util.List;

public class Blob {


    public ColorBlobLocatorProcessor CameraSetUp(String PixelColor, int XCameraResolutionHeight, int YCameraResolutionWidth){

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                //.setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        switch (PixelColor){
            case "Yellow":
                colorLocator = new ColorBlobLocatorProcessor.Builder()
                        .setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match// Smooth the transitions between different colors in image
                        .build();
                break;
            case "Blue":
                colorLocator = new ColorBlobLocatorProcessor.Builder()
                        .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match// Smooth the transitions between different colors in image
                        .build();
                break;
            case "Red":
                colorLocator = new ColorBlobLocatorProcessor.Builder()
                        .setTargetColorRange(ColorRange.RED)         // use a predefined color match// Smooth the transitions between different colors in image
                        .build();
                break;
        }

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(XCameraResolutionHeight, YCameraResolutionWidth))
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        opMode.telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        opMode.telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        return colorLocator;
    }

    public ArrayList<Double> GetSampleCenter(ColorBlobLocatorProcessor colorLocator, ArrayList<Double> SampleCenter){
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);

        opMode.telemetry.addLine(" Area Density Aspect  Center");

        // Display the size (area) and center location for each Blob.
        for(ColorBlobLocatorProcessor.Blob b : blobs)
        {
            RotatedRect boxFit = b.getBoxFit();
            opMode.telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                    b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
            SampleCenter.add(0, boxFit.center.x);
            SampleCenter.add(1, boxFit.center.y);

        }

        opMode.telemetry.update();
        //opMode.sleep(50);
        return SampleCenter;
    }
}
