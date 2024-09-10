package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.individual_components.Grabber;
import org.firstinspires.ftc.teamcode.individual_components.Lift;
import org.firstinspires.ftc.teamcode.individual_components.Pivot;
import org.firstinspires.ftc.teamcode.individual_components.Wrist;
import org.firstinspires.ftc.teamcode.profiles_and_base_settings.Profiles;
import org.firstinspires.ftc.teamcode.profiles_and_base_settings.Settings;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
@Autonomous(name = "SoupCopmode_RedBack", group = "Autonomous")
public class Trajectory1 extends BasicMecanumOpMode{
    double x;
    double y;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "Red.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    //
    // private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Prop",
    };
    private ElapsedTime frameTimer = new ElapsedTime();

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    int propLocation;
    private ElapsedTime elapsedTime = new ElapsedTime();
    int white = 0;
    int white1 = 0;
    double end = 30;
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();

        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
    @Override
    public void runOpMode() {

        Profiles driverProfiles = new Profiles(this); // creates an instance of the profiles class to access the date because I could figure out how to access it otherwise even though I think it should be possible
        Settings activeSettings = driverProfiles.defaultProfile; // selects the active setting that will be used in the rest of the code

        Grabber grabber = new Grabber(this, activeSettings);

        Lift lift = new Lift(this, activeSettings);


        Pivot pivot = new Pivot(this, activeSettings);
        pivot.debugModeEnabled = true;

        Wrist wrist = new Wrist(this, activeSettings);

        wrist.debugMode = true;


        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        frameTimer.reset();

        double deltaTime = 0;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                deltaTime = frameTimer.seconds(); //gets the time since the start of last frame and then resets the timer
                telemetry.addData("deltaTime ", deltaTime);
                frameTimer.reset();
                lift.runLiftPID(deltaTime);
                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, -63, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        /*TODO
        Replace junk numbers if statement
        Integrate TF model into code (Maybe done?)
        Test everything
         */
        Trajectory moveToSpikes = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(12, -36), Math.toRadians(propLocation * 90))
                .build();
        Trajectory moveToSpikes_Right = drive.trajectoryBuilder(moveToSpikes.end())
                .splineTo(new Vector2d(24, -40), Math.toRadians(90))
                .build();
        Trajectory moveToBackground = drive.trajectoryBuilder(moveToSpikes.end())
                .splineTo(new Vector2d(48, -36), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(48, -42 + (propLocation * 6)))
                .build();

        Trajectory moveToBackstage = drive.trajectoryBuilder(moveToBackground.end())
                .splineTo(new Vector2d(60, -60), Math.toRadians(0))
                .build();


        waitForStart();

        if(isStopRequested()) return;

        //Camera; Identify Team Prop location
        if (x >= 0) {   //0 is right, 1 is center, 2 is left; reading x still uses junk numbers
            propLocation = 0;
        } else if (x <= 640) {
            propLocation = 2;
        } else {
            propLocation = 1;
        }

        if (propLocation == 0) {
            drive.followTrajectory(moveToSpikes_Right);
        } else {
            drive.followTrajectory(moveToSpikes);
        }
        
        //Lift; Place Purple Pixel

        pivot.setTargetPositionDeg(30);
        wrist.wristServo1.setAngle(-30);
        wrist.wristServo2.setAngle(0);
        Grabber.grabber1_Grabbing(false);

        
        drive.followTrajectory(moveToBackground);
        //Lift; Place Yellow Pixel

        pivot.setTargetPositionDeg(120);
        wrist.wristServo1.setAngle(-30);
        wrist.wristServo2.setAngle(0);
        Grabber.grabber2_Grabbing(false);


        drive.followTrajectory(moveToBackstage);
        }




}





