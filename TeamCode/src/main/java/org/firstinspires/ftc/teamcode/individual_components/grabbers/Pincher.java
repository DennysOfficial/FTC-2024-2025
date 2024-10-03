package org.firstinspires.ftc.teamcode.individual_components.grabbers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;

public class Pincher {

    LinearOpMode opMode;
    RobotConfig config;


    Servo pinchServo;
    float pincherClosedPosition = 0.431f;
    float pincherOpenPosition = 0.6778f;

    Servo wristServo;


    public Pincher(LinearOpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;
        pinchServo = opMode.hardwareMap.get(Servo.class, "Pinch");
        wristServo = opMode.hardwareMap.get(Servo.class, "GrabWrist");
    }


    public void directControl(double deltaTime) {

        updatePincher();
        wristServo.setPosition(wristServo.getPosition() + opMode.gamepad2.right_stick_y * deltaTime * 0.5);
        opMode.telemetry.addData("wristPosition", wristServo.getPosition());

    }

    boolean previousPinchButton = false;

    void updatePincher() {
        if (config.getPinchButton() && (config.getPinchButton()) != previousPinchButton)
            togglePincher();
        previousPinchButton = config.getPinchButton();

        switch (pincherState){
            case Closed:
                pinchServo.setPosition(pincherClosedPosition);
                break;
            case Open:
                pinchServo.setPosition(pincherOpenPosition);
                break;
        }


    }

    enum PincherState{
        Closed,
        Open
    }
    PincherState pincherState = PincherState.Closed;
    void togglePincher(){

        if (pincherState == PincherState.Open)
            pincherState = PincherState.Closed;

        else
            pincherState = PincherState.Open;
    }


}
