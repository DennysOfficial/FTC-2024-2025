package org.firstinspires.ftc.teamcode.Blob;

import android.util.Size;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "newOPMODE", group = "Linear OpMode")

public class realOpMode extends LinearOpMode {
    public void runOpMode() {

        double XCameraResolutionHeight = 640;
        double YCameraResolutionWidth = 480;
        double CameraAngle = 54;
        double liftAngle = 90;
        double liftExtension = 0;
        String PixelColorBlue = "Blue";
        String PixelColorRed = "Red";
        String PixelColorYellow = "Yellow";

        Blob NewBlob = new Blob(this);

        OPBLOB thing = new OPBLOB(this);

        BestSampleToPickUpAlgorithm idk = new BestSampleToPickUpAlgorithm();


        // this RobotPose needs to be updated with robot pose from otos or roadrunner or whatever
        SparkFunOTOS.Pose2D RobotPose = new SparkFunOTOS.Pose2D();
        RobotPose.x = 0;
        RobotPose.y = 0;
        RobotPose.h = 0;


        ColorBlobLocatorProcessor colorLocatorBlue ;
        ColorBlobLocatorProcessor colorLocatorRed ;
        //ColorBlobLocatorProcessor colorLocatorYellow ;


        //waitForStart();
        colorLocatorBlue = NewBlob.CameraSetUp(PixelColorBlue, (int) XCameraResolutionHeight, (int) YCameraResolutionWidth);
        colorLocatorRed = NewBlob.CameraSetUp(PixelColorBlue, (int) XCameraResolutionHeight, (int) YCameraResolutionWidth);
        //colorLocatorYellow = NewBlob.CameraSetUp(PixelColorBlue, (int) XCameraResolutionHeight, (int) YCameraResolutionWidth);

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocatorBlue)
                //.addProcessor(colorLocatorRed)
                //.addProcessor(colorLocatorYellow)
                .setCameraResolution(new Size((int) XCameraResolutionHeight, (int) YCameraResolutionWidth))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();




        while (opModeInInit()) {
            int sampleCount = 1;
            int colorNum = 0;

            if (gamepad1.dpad_right) {
                sampleCount += 1;
                idk.displaySamplePosition(sampleCount);
            }

            if (gamepad1.dpad_left) {
                sampleCount -= 1;
                if (sampleCount < 1) {
                    sampleCount = 1;
                }
                idk.displaySamplePosition(sampleCount);
            }


            if (gamepad1.a) {

                double[][] finalCopy;
                finalCopy = idk.getDeepCopy();
                for (int i = 1; i < finalCopy.length/4 - 1; i++){
                    idk.displaySamplePosition(i);
                }
            }
            //telemetry.addData("Sample Number", sampleCount);
            if (gamepad1.dpad_up){
                colorNum += 1;
                if (colorNum > 1){
                    colorNum = 0;
                }

            }

            switch (colorNum){
                case 0:
                    thing.OPBlob(colorLocatorBlue, XCameraResolutionHeight, YCameraResolutionWidth, CameraAngle, liftAngle, liftExtension, RobotPose);
                    break;
                case 1:
                    thing.OPBlob(colorLocatorRed, XCameraResolutionHeight, YCameraResolutionWidth, CameraAngle, liftAngle, liftExtension, RobotPose);
                    break;

            }


            //thing.OPBlob(colorLocatorBlue, XCameraResolutionHeight, YCameraResolutionWidth, CameraAngle, liftAngle, liftExtension, RobotPose);
            //thing.OPBlob(colorLocatorRed, XCameraResolutionHeight, YCameraResolutionWidth, CameraAngle, liftAngle, liftExtension, RobotPose);
            //thing.OPBlob(colorLocatorYellow, XCameraResolutionHeight, YCameraResolutionWidth, CameraAngle, liftAngle, liftExtension, RobotPose);

            //sleep(30);

            telemetry.update();


        }
    }
}
