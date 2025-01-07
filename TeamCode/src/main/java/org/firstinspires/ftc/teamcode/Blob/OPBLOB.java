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

@TeleOp(name = "OPMODE", group = "Linear OpMode")

public class OPBLOB extends LinearOpMode {
    public void runOpMode() {

        double liftAngle = 0;
        double liftExtension = 0;

        String PixelColor = "Blue";
        double XCameraResolutionHeight = 640;
        double YCameraResolutionWidth = 480;
        double CameraAngle = 54;


        CameraData cameraData = new CameraData();
        BestSampleToPickUpAlgorithm BSample = new BestSampleToPickUpAlgorithm(this);

        cameraData.liftAngle = liftAngle;
        cameraData.liftExtension =liftExtension;
        cameraData.xResolution = (int)XCameraResolutionHeight;
        cameraData.yResolution = (int)YCameraResolutionWidth;
        cameraData.pitchAngle = CameraAngle + cameraData.liftAngle;


        Blob NewBlob = new Blob(this);
        ColorBlobLocatorProcessor colorLocator = NewBlob.CameraSetUp(PixelColor, (int) XCameraResolutionHeight, (int) YCameraResolutionWidth);

        //SparkFunOTOS.Pose2D SampleCenter;

        //Vector3D VectorToCam;

        SparkFunOTOS.Pose2D RobotPose = new SparkFunOTOS.Pose2D();
        //SparkFunOTOS.Pose2D SamplePose = new SparkFunOTOS.Pose2D();
        SparkFunOTOS.Pose2D EachPose = new SparkFunOTOS.Pose2D();
        //ArrayList<Double> CameraOffsets = new ArrayList<>();
        List<Pose2D> AllSampleGobalPositons = new ArrayList<>();

        //AllSamplePoses = NewBlob.GetSampleCenter(colorLocator, AllSamplePoses);
        //loop all bellow

        while (opModeIsActive() || opModeInInit()) {
            //SampleCenter = NewBlob.GetSampleCenter(colorLocator, poses);

            List<Pose2D> poses = new ArrayList<>();
            poses = NewBlob.GetSampleCenter(colorLocator, poses);


            //ArrayList<Double> VectorToRobot = new ArrayList<Double>();
            //NewBlob.VectorToRobot(VectorToRobot, RobotX,  RobotY, RobotHeading);

            NewBlob.CameraOffsetSetup(cameraData);
            cameraData.positionOnRobot = NewBlob.CamOffsetVectorFromOrgin(RobotPose, cameraData);

            //loop through all of the different poses of the samples get a pose of each then add to a list
            for (Pose2D pose : poses){
                EachPose.x = pose.getX(DistanceUnit.INCH);
                EachPose.y = pose.getY(DistanceUnit.INCH);
                EachPose.h = pose.getHeading(AngleUnit.RADIANS);

                //SamplePose = NewBlob.SampleLocation(EachPose, VectorToCam, cameraData, SamplePose, RobotPose);
                AllSampleGobalPositons.add(NewBlob.SampleLocation(EachPose, cameraData.positionOnRobot, cameraData, RobotPose));

            }

            // no clue if this works or how it works
            //AllSampleGobalPositons.forEach(component -> telemetry.addLine(component.toString()));

            //List<Pose2D> newSamplelist = new ArrayList<>();
            //newSamplelist = BSample.NewListOfSamples(AllSampleGobalPositons);
            BSample.NewListOfSamples(AllSampleGobalPositons);
            BSample.normToSubCorner(RobotPose);
            BSample.difference();
            BSample.deepCopy.forEach(component -> telemetry.addLine(component.toString()));





            //telemetry.addData("Sample center", poses.
            //telemetry.addData("Sample center", poses.y);
            telemetry.addData("Vectortocam", cameraData.positionOnRobot.getX());
            telemetry.addData("Vectortocam", cameraData.positionOnRobot.getY());
            telemetry.addData("Vectortocam", cameraData.positionOnRobot.getZ());

            telemetry.addData("CameraX" , cameraData.xOffset);
            telemetry.addData("Cameray" , cameraData.yOffset);
            telemetry.addData("Cameraz" , cameraData.zOffset);
            telemetry.addData("Camerah" , cameraData.CameraOffsetAngle);
            //telemetry.addData("x", SamplePose.x);
            //telemetry.addData("y", SamplePose.y);

            telemetry.update();

        }

    }
}
