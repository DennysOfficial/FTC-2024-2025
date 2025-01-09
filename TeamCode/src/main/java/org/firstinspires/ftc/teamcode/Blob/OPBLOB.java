package org.firstinspires.ftc.teamcode.Blob;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.ArrayList;
import java.util.List;

//@TeleOp(name = "OPMODE", group = "Linear OpMode")

public class OPBLOB {

    LinearOpMode opMode;
    public OPBLOB(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void OPBlob(ColorBlobLocatorProcessor colorLocator, double XCameraResolutionHeight, double YCameraResolutionWidth, double CameraAngle, double liftAngle, double liftExtension, SparkFunOTOS.Pose2D RobotPose) {






        CameraData cameraData = new CameraData();
        BestSampleToPickUpAlgorithm BSample = new BestSampleToPickUpAlgorithm();

        cameraData.liftAngle = liftAngle;
        cameraData.liftExtension = liftExtension;
        cameraData.xResolution = (int) XCameraResolutionHeight;
        cameraData.yResolution = (int) YCameraResolutionWidth;
        cameraData.pitchAngle = CameraAngle + cameraData.liftAngle;

        Blob NewBlob = new Blob(opMode);


        //SparkFunOTOS.Pose2D RobotPose = new SparkFunOTOS.Pose2D();

        SparkFunOTOS.Pose2D EachPose = new SparkFunOTOS.Pose2D();

        List<Pose2D> AllSampleGobalPositons = new ArrayList<>();



        List<Pose2D> poses = new ArrayList<>();

        poses = NewBlob.GetSampleCenter(colorLocator, poses);




        NewBlob.CameraOffsetSetup(cameraData);
        cameraData.positionOnRobot = NewBlob.CamOffsetVectorFromOrgin(RobotPose, cameraData);

        //loop through all of the different poses of the samples get a pose of each then add to a list
        for (Pose2D pose : poses) {
            EachPose.x = pose.getX(DistanceUnit.INCH);
            EachPose.y = pose.getY(DistanceUnit.INCH);
            EachPose.h = pose.getHeading(AngleUnit.RADIANS);

            double[] numbers;

            numbers = NewBlob.SampleLocation(EachPose, cameraData.positionOnRobot, cameraData, RobotPose);
            opMode.telemetry.addData(" ", numbers[0]);
            AllSampleGobalPositons.add(new Pose2D(DistanceUnit.INCH,numbers[0],numbers[1],AngleUnit.RADIANS,numbers[2]));

        }


        // no clue if this works or how it works
        //AllSampleGobalPositons.forEach(component -> telemetry.addLine(component.toString()));

        BSample.NewListOfSamples(AllSampleGobalPositons);
        //BSample.normToSubCorner(RobotPose);
        //BSample.difference();
        AllSampleGobalPositons.forEach(component -> opMode.telemetry.addLine(component.toString()));
        BSample.deepCopy.forEach(component -> opMode.telemetry.addLine(component.toString()));




        opMode.telemetry.addData("Vectortocam", cameraData.positionOnRobot.getX());
        opMode.telemetry.addData("Vectortocam", cameraData.positionOnRobot.getY());
        opMode.telemetry.addData("Vectortocam", cameraData.positionOnRobot.getZ());

        opMode.telemetry.addData("CameraX", cameraData.xOffset);
        opMode.telemetry.addData("Cameray", cameraData.yOffset);
        opMode.telemetry.addData("Cameraz", cameraData.zOffset);
        opMode.telemetry.addData("Camerah", cameraData.CameraOffsetAngle);


        opMode.telemetry.update();

    }




}
