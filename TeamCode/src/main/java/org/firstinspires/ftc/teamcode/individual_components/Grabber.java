package org.firstinspires.ftc.teamcode.individual_components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.profiles_and_base_settings.Settings;

public class Grabber {

    boolean debugMode = false;

    LinearOpMode opMode;
    Settings settings;

    static Servo grabberServo1;
    static Servo grabberServo2;

    static double[] grabber1Limits = {0,1}; // {open, closed}
    static double[] grabber2Limits = {1,0};


    public Grabber(LinearOpMode opModeTemp, Settings settingsTemp){
        opMode = opModeTemp;
        settings = settingsTemp;

        grabberServo1 = opMode.hardwareMap.get(Servo.class,"Grabber Servo-1");
        grabberServo2 = opMode.hardwareMap.get(Servo.class,"Grabber Servo-2");
    }

    boolean grabber1PreviousState = false;
    boolean grabber2PreviousState = false;
    public void manualMove(){

        if(!grabber1PreviousState && settings.getGrabber1()){  // if the button was previously not pressed and is now pressed

            if(grabberServo1.getPosition() == grabber1Limits[0]) // toggles state of grabber if previous conditions are met
                grabberServo1.setPosition(grabber1Limits[1]);
            else
                grabberServo1.setPosition(grabber1Limits[0]);

        }
        grabber1PreviousState = settings.getGrabber1(); // stores the state of the button for the next loop

        if(!grabber2PreviousState && settings.getGrabber2()){

            if(grabberServo2.getPosition() == grabber2Limits[0])
                grabberServo2.setPosition(grabber2Limits[1]);
            else
                grabberServo2.setPosition(grabber2Limits[0]);

        }
        grabber2PreviousState = settings.getGrabber2();

        if(debugMode){
            opMode.telemetry.addLine();
            opMode.telemetry.addData("grabber 1 status","position: " + grabberServo1.getPosition());
            opMode.telemetry.addData("grabber 2 status","position: " + grabberServo2.getPosition());
        }

    }


    public static void grabber1_Grabbing(boolean pos1) {   //true = grabbing; false = not grabbing
        if(pos1){
            grabberServo1.setPosition(grabber1Limits[0]);
        }else{
            grabberServo1.setPosition(grabber1Limits[1]);
        }
    }
    public static void grabber2_Grabbing(boolean pos2) {   //true = grabbing; false = not grabbing
        if(pos2){
            grabberServo2.setPosition(grabber2Limits[0]);
        }else{
            grabberServo2.setPosition(grabber2Limits[1]);
        }
    }

    public void setGrabber1Grabbing(boolean pos1) {   //true = grabbing; false = not grabbing
        if(pos1){
            grabberServo1.setPosition(grabber1Limits[0]);
        }else{
            grabberServo1.setPosition(grabber1Limits[1]);
        }
    }
    public void setGrabber2Grabbing(boolean pos2) {   //true = grabbing; false = not grabbing
        if(pos2){
            grabberServo2.setPosition(grabber2Limits[0]);
        }else{
            grabberServo2.setPosition(grabber2Limits[1]);
        }
    }
}
