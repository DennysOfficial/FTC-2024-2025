package org.firstinspires.ftc.teamcode.Blob;

//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Size;

//import com.acmerobotics.roadrunner.Vector2dDual;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;


import java.util.ArrayList;
import java.util.List;

public class Blob {

    LinearOpMode opMode;

    public Blob(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public ColorBlobLocatorProcessor CameraSetUp(String PixelColor, int XCameraResolutionHeight, int YCameraResolutionWidth) {

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();


        switch (PixelColor) {
            case "Yellow":
                colorLocator = new ColorBlobLocatorProcessor.Builder()
                        .setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match// Smooth the transitions between different colors in image
                        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                        .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                        .setDrawContours(true)                        // Show contours on the Stream Preview
                        .setBlurSize(5)
                        .build();
                break;
            case "Blue":
                 colorLocator = new ColorBlobLocatorProcessor.Builder()
                        .setTargetColorRange(ColorRange.BLUE)
                        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                        .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                        .setDrawContours(true)                        // Show contours on the Stream Preview
                        .setBlurSize(5)                               // Smooth the transitions between different colors in image

                        // use a predefined color match// Smooth the transitions between different colors in image
                        .build();
                break;
            case "Red":
                 colorLocator = new ColorBlobLocatorProcessor.Builder()
                        .setTargetColorRange(ColorRange.RED)// use a predefined color match// Smooth the transitions between different colors in image
                        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                        .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))  // search central 1/4 of camera view
                        .setDrawContours(true)                        // Show contours on the Stream Preview
                        .setBlurSize(5)
                        .build();
                break;
        }



        opMode.telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        opMode.telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        return colorLocator;
    }


    public List<Pose2D> GetSampleCenter(ColorBlobLocatorProcessor colorLocator, List<Pose2D> poses) {
        SparkFunOTOS.Pose2D SampleCenter = new SparkFunOTOS.Pose2D();

        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(80, 5000, blobs);

        opMode.telemetry.addLine(" Area Density Aspect  Center");

        // Display the size (area) and center location for each Blob.
        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            RotatedRect boxFit = b.getBoxFit();
            opMode.telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                    b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
            //List<Pose2D> poses = new ArrayList<>();
            poses.add(new Pose2D(DistanceUnit.INCH, boxFit.center.x,boxFit.center.y, AngleUnit.RADIANS, boxFit.angle));
            //SampleCenter.x = (boxFit.center.x);
            //SampleCenter.y = (boxFit.center.y);
            //SampleCenter.h = (boxFit.angle);


        }
        //opMode.telemetry.update();
        //sleep(50);
        return poses;
    }

    public void CameraOffsetSetup(CameraData cameraData) {
        //double OtosToLiftPivotX = 0; // measured
        //double OtosToLiftPivotY = 0; // measured
        //double OtosToLiftPivotZ = 8;// measured
        //double distanceFromAxisCamIsMounted = 0;
        //double CamYOffset = Math.cos(cameraData.liftAngle) * cameraData.liftExtension + distanceFromAxisCamIsMounted;
        //double CamZOffset = Math.sin(cameraData.liftAngle) * cameraData.liftExtension + distanceFromAxisCamIsMounted;

        //cameraData.yOffset = CamYOffset + OtosToLiftPivotY;
        //cameraData.zOffset = CamZOffset + OtosToLiftPivotZ;
        cameraData.yOffset = 10;
        cameraData.zOffset = 12;
        cameraData.xOffset = 2;

        //cameraData.xOffset = OtosToLiftPivotX;

        //CameraOffsets.add(0, cameraData.xOffset);
        //CameraOffsets.add(1, cameraData.yOffset);
        //CameraOffsets.add(2, cameraData.zOffset);
        cameraData.CameraOffsetAngle = Math.toDegrees(Math.tanh(cameraData.xOffset / cameraData.yOffset));
        //CameraOffsets.add(3, cameraData.CameraOffsetAngle);
    }

    public ArrayList<Double> VectorToRobot(ArrayList<Double> VectorToRobot, double RobotX, double RobotY, double RobotHeading) {

        VectorToRobot.add(0, RobotX);
        VectorToRobot.add(1, RobotY);
        VectorToRobot.add(3, RobotHeading);
        return VectorToRobot;
    }

    public Vector3D CamOffsetVectorFromOrgin( SparkFunOTOS.Pose2D Vector,  CameraData cameraData) {

        cameraData.headingAngleOffset = Math.toRadians(cameraData.CameraOffsetAngle) + Math.toRadians(Vector.h);
        double MagOffset = Math.sqrt(Math.pow(2, cameraData.xOffset) + Math.pow(2, cameraData.yOffset));
        //VectorCam.add(0, MagOffset* Math.cos(angle));
        //VectorCam.add(1, MagOffset* Math.sin(angle));
        //VectorCam.add(2, CameraOffsets.get(2));


        return new Vector3D( (MagOffset * Math.sin(cameraData.headingAngleOffset) + Vector.x) , MagOffset * Math.cos(cameraData.headingAngleOffset) + Vector.y , cameraData.zOffset);
    }



    public double[] SampleLocation(SparkFunOTOS.Pose2D sampleCenter, Vector3D vectorToCam, CameraData cameraData, SparkFunOTOS.Pose2D robotPose) {

        //double VAngle;
        double SampleDistanceFromCam;
        double SampleLRFromCam;
        //angle of pixel from center
        double sampleX;
        double sampleY;
        double sampleH;

        double screenCordX = sampleCenter.x - cameraData.xResolution / 2.0;
        double normalizedScreenCordX = screenCordX / (cameraData.xResolution / 2.0);
        double HAngle = normalizedScreenCordX * (cameraData.HFOV / 2);

        double screenCordY = sampleCenter.y - cameraData.yResolution / 2.0;
        double normalizedScreenCordY = screenCordY / (cameraData.yResolution / 2.0);
        double VAngle = normalizedScreenCordY * (cameraData.VFOV / 2) * -1;
        opMode.telemetry.addData("screencordX", screenCordX);
        opMode.telemetry.addData("screencordY", screenCordY);
        //VAngle = (sampleCenter.y + (cameraData.yResolution / 4.0) - (cameraData.yResolution / 2.0)) / ((cameraData.yResolution / 2.0) * cameraData.VFOV / 2);
        opMode.telemetry.addData("vAngle", VAngle);
        VAngle += cameraData.pitchAngle;
        opMode.telemetry.addData("vAngle", VAngle);
        //this not work
        //SampleDistanceFromCam = vectorToCam.getZ() * Math.tan(Math.toDegrees(VAngle))*-1  ;
        SampleDistanceFromCam = Math.sin(Math.toRadians(VAngle))/ Math.sin(Math.toRadians(90-VAngle)) * vectorToCam.getZ();
        opMode.telemetry.addData("SampleD", SampleDistanceFromCam);


        // double CameraLenseToSample = Math.sqrt(Math.pow(2, SampleDistanceFromCam) + Math.pow(2, vectorToCam.getZ()));
        // SampleLRFromCam = SampleDistanceFromCam * Math.tan(Math.toRadians(HAngle));
        SampleLRFromCam = Math.sin(Math.toRadians(HAngle))/ Math.sin(Math.toRadians(90 -HAngle)) * SampleDistanceFromCam;
        // SampleLRFromCam = Math.tan(Math.toRadians(HAngle)) * CameraLenseToSample;
        // vector rotation
        opMode.telemetry.addData("thing",SampleDistanceFromCam);
        opMode.telemetry.addData("thing2",SampleLRFromCam);

        sampleX = SampleDistanceFromCam * Math.cos(Math.toRadians(robotPose.h )) - SampleLRFromCam * Math.sin(Math.toRadians(robotPose.h ));
        sampleY = SampleLRFromCam * Math.cos(Math.toRadians(robotPose.h )) +  SampleDistanceFromCam * Math.sin(Math.toRadians(robotPose.h ));
        sampleH = sampleCenter.h;
        opMode.telemetry.addData("hAngle", HAngle);
        opMode.telemetry.addData("x", sampleX);
        opMode.telemetry.addData("y", sampleY);
        //opMode.telemetry.addData("xx", vectorToCam.getX());
        //opMode.telemetry.addData("yy", vectorToCam.getY());
        opMode.telemetry.addData("z", vectorToCam.getZ());

        //sampleX += vectorToCam.getX();
        //sampleY += vectorToCam.getY();



        return new double[]{sampleX, sampleY, sampleH};
    }

}
