package org.firstinspires.ftc.teamcode.Blob;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

@TeleOp(name = "OPMODE", group = "Linear OpMode")

public class OPBLOB extends LinearOpMode {
    public void runOpMode() {

        String PixelColor = "Blue";
        double XCameraResolutionHeight = 640;
        double YCameraResolutionWidth = 480;
        double CameraAngle = 54;


        CameraData cameraData = new CameraData();
        cameraData.xResolution = (int)XCameraResolutionHeight;
        cameraData.yResolution = (int)YCameraResolutionWidth;
        cameraData.pitchAngle = CameraAngle;


        Blob NewBlob = new Blob(this);
        ColorBlobLocatorProcessor colorLocator = NewBlob.CameraSetUp(PixelColor, (int) XCameraResolutionHeight, (int) YCameraResolutionWidth);

        SparkFunOTOS.Pose2D SampleCenter;

        Vector3D VectorToCam;

        SparkFunOTOS.Pose2D RobotPose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D SamplePose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D EachPose = new SparkFunOTOS.Pose2D();
        ArrayList<Double> CameraOffsets = new ArrayList<>();

        List<Pose2D> AllSamplePoses = new ArrayList<>();
        AllSamplePoses = NewBlob.GetSampleCenter(colorLocator, AllSamplePoses);
        //loop all bellow

        while (opModeIsActive() || opModeInInit()) {
            //SampleCenter = NewBlob.GetSampleCenter(colorLocator, poses);

            List<Pose2D> poses = new ArrayList<>();
            poses = NewBlob.GetSampleCenter(colorLocator, poses);


            //ArrayList<Double> VectorToRobot = new ArrayList<Double>();
            //NewBlob.VectorToRobot(VectorToRobot, RobotX,  RobotY, RobotHeading);

            CameraOffsets = NewBlob.CameraOffsetSetup(CameraOffsets, cameraData);

            VectorToCam = NewBlob.CamOffsetVectorFromOrgin(CameraOffsets, RobotPose);

            //loop through all of the different poses of the samples get a pose of each then add to a list
            for (Pose2D pose : poses){
                EachPose.x = pose.getX(DistanceUnit.INCH);
                EachPose.y = pose.getY(DistanceUnit.INCH);
                EachPose.h = pose.getHeading(AngleUnit.RADIANS);

                SamplePose = NewBlob.SampleLocation(EachPose, VectorToCam, cameraData, SamplePose, RobotPose);
                AllSamplePoses.


            }


            //telemetry.addData("Sample center", poses.
            //telemetry.addData("Sample center", poses.y);
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
