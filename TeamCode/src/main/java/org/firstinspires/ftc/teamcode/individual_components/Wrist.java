package org.firstinspires.ftc.teamcode.individual_components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.misc.AngleServo;
import org.firstinspires.ftc.teamcode.profiles_and_base_settings.Settings;

import java.sql.Struct;

public class Wrist {

    public AngleServo wristServo1; // numbered in ascending order from the chassis of the robot out
    public AngleServo wristServo2;
    AngleServo wristServo2IntakeGrabber;
    AngleServo wristServo2GroundGrabber;


    public Settings.Grabbers activeGrabber;

    public boolean debugMode = false;

    // Deg limits are a maximum travel for 0 - 1 input range
    // pivot arm down and in line with the lift is 0deg
    //counterclockwise when looking at the robot from the right is positive

    LinearOpMode opMode;
    Settings settings;


    public Wrist(LinearOpMode opMode, Settings settings) {
        this.opMode = opMode;
        this.settings = settings;
        this.activeGrabber = settings.activeGrabber;

        // Deg max/min are a maximum travel for 0 - 1 input not limits that stop the servo from moving


        //counterclockwise when looking at the robot from the right is positive for wrist 1
        // wrist1 in line with the pivot arm is 0deg
        wristServo1 = new AngleServo(opMode, "wrist-1", -96, 96);

        //wristServo1.setDirection(Servo.Direction.REVERSE);

        //clockwise when looking at the grabber from the top is positive for wrist 2
        //grabber pointed forward is 0deg
        wristServo2IntakeGrabber = new AngleServo(opMode, "wrist-2", -230, 40);
        wristServo2GroundGrabber = new AngleServo(opMode, "wrist-2", -210, 40);

        wristServo2IntakeGrabber.setDirection(Servo.Direction.REVERSE);
        wristServo2GroundGrabber.setDirection(Servo.Direction.REVERSE);

        switch (activeGrabber){
            case groundGrabber:
                wristServo2 = wristServo2GroundGrabber;
                return;
            case intakeGrabber:
                wristServo2 = wristServo2IntakeGrabber;
                return;
        }
    }

    public void servo1ManualMove(double deltaTime) {
        wristServo1.setAngle(wristServo1.getAngle() + settings.getWrist1Stick() * settings.wrist1Sensitivity * deltaTime);

        if(debugMode){
            opMode.telemetry.addData("Wrist servo 1",wristServo1.getAngle());
        }
    }

    public void servo2ManualMove(double deltaTime) {
        wristServo2.setAngle(wristServo2.getAngle() + settings.getWrist2Stick() * settings.wrist2Sensitivity * deltaTime);
        if(debugMode){
            opMode.telemetry.addData("Wrist servo 2",wristServo2.getAngle());
        }
    }


}
