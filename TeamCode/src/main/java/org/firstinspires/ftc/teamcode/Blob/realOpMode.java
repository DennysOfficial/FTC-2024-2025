package org.firstinspires.ftc.teamcode.Blob;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

@TeleOp(name = "newOPMODE", group = "Linear OpMode")

public class realOpMode extends LinearOpMode {
    public void runOpMode() {

        double XCameraResolutionHeight = 320;
        double YCameraResolutionWidth = 240;
        double CameraAngle = 54;
        double liftAngle = 90;
        double liftExtension = 0;
        String PixelColor = "Blue";

        Blob NewBlob = new Blob(this);

        OPBLOB thing = new OPBLOB(this);


        // this RobotPose needs to be updated with robot pose from otos or roadrunner or whatever
        SparkFunOTOS.Pose2D RobotPose = new SparkFunOTOS.Pose2D();


        ColorBlobLocatorProcessor colorLocator ;

        //waitForStart();
        colorLocator = NewBlob.CameraSetUp(PixelColor, (int) XCameraResolutionHeight, (int) YCameraResolutionWidth);

        while (opModeInInit()){

        thing.OPBlob(colorLocator, XCameraResolutionHeight, YCameraResolutionWidth, CameraAngle, liftAngle, liftExtension, RobotPose);

        sleep(30);

        }
    }
}
