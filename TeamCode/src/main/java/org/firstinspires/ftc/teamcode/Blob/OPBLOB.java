package org.firstinspires.ftc.teamcode.Blob;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.ArrayList;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

@TeleOp(name = "OPMODE", group = "Linear OpMode")

public class OPBLOB extends LinearOpMode {
    public void runOpMode() {

        String PixelColor = "Blue";
        double XCameraResolutionHeight = 640;
        double YCameraResolutionWidth = 480;
        double CameraAngle = 30;
        ArrayList<Double> Camera = new ArrayList<>();
        Camera.add(0,XCameraResolutionHeight);
        Camera.add(1,YCameraResolutionWidth);
        Camera.add(2,CameraAngle);

        Blob.CameraData cameraData = new Blob.CameraData();

        Blob NewBlob = new Blob(this);
        ColorBlobLocatorProcessor colorLocator = NewBlob.CameraSetUp(PixelColor, (int) XCameraResolutionHeight, (int) YCameraResolutionWidth);

        SparkFunOTOS.Pose2D SampleCenter;

        Vector3D VectorToCam;

        SparkFunOTOS.Pose2D RobotPose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D SamplePose = new SparkFunOTOS.Pose2D();
        ArrayList<Double> CameraOffsets = new ArrayList<>();
        //loop all bellow

        while (opModeIsActive()) {
            SampleCenter = NewBlob.GetSampleCenter(colorLocator);

            //ArrayList<Double> VectorToRobot = new ArrayList<Double>();
            //NewBlob.VectorToRobot(VectorToRobot, RobotX,  RobotY, RobotHeading);

            CameraOffsets = NewBlob.CameraOffsetSetup(CameraOffsets);

            VectorToCam = NewBlob.CamOffsetVectorFromOrgin(CameraOffsets, SamplePose);

            SamplePose = NewBlob.SampleLocation(SampleCenter, VectorToCam, Camera, SamplePose, RobotPose);

            telemetry.addData("Sample center", SampleCenter.x);
            telemetry.addData("Sample center", SampleCenter.y);
            telemetry.addData("Vectortocam", VectorToCam.getX());
            telemetry.addData("Vectortocam", VectorToCam.getY());
            telemetry.addData("Vectortocam", VectorToCam.getZ());

            telemetry.addData("CameraX" , (CameraOffsets.get(0)));
            telemetry.addData("Cameray" , (CameraOffsets.get(1)));
            telemetry.addData("Cameraz" , (CameraOffsets.get(2)));
            telemetry.addData("Camerah" , (CameraOffsets.get(3)));
            telemetry.addData("x", SamplePose.x);
            telemetry.addData("y", SamplePose.y);

            telemetry.update();

        }

    }
}
