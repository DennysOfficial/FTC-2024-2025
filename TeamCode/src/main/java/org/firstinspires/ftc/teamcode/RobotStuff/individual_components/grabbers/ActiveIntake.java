package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Timer;

public class ActiveIntake {

    OpMode opMode;
    RobotConfig config;


    CRServo spinnyServo;

    Servo flapServo;

    float intakeSpeed = -1;
    float outtakeSpeed = 1;

    double flapOpen = 0;
    double flapClosed = 0.9;


    public ActiveIntake(OpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;
        spinnyServo = opMode.hardwareMap.get(CRServo.class, config.deviceConfig.grabberServo);
        flapServo = opMode.hardwareMap.get(Servo.class, config.deviceConfig.flapServo);
    }

    Timer stopTimer = null;
    Timer closeFlapTimer = null;

    public void update() {
        if (stopTimer != null && stopTimer.done()) {
            stopTimer = null;
            stop();
        }

        if (closeFlapTimer != null && closeFlapTimer.done()) {
            closeFlapTimer = null;
            closeFlap();
        }
    }

    public void directControl() {
        wheelControl();
        flapControl();
    }

    public void wheelControl() {
        spinnyServo.setPower(0);

        if (config.inputMap.getIntakeButton())
            spinnyServo.setPower(intakeSpeed);

        else if (config.inputMap.getOuttakeButton())
            spinnyServo.setPower(outtakeSpeed);
    }

    public void flapControl() {

        if (config.inputMap.getFlapButton()) {
            flapServo.setPosition(flapOpen);
            return;
        }

        flapServo.setPosition(flapClosed);
    }

    public void intake() {
        spinnyServo.setPower(intakeSpeed);
    }


    public void intakeForDuration(double durationSeconds) {
        intake();
        stopTimer = new Timer(durationSeconds);
    }

    public void outtake() {
        spinnyServo.setPower(outtakeSpeed);
    }

    public void outtakeForDuration(double durationSeconds) {
        outtake();
        stopTimer = new Timer(durationSeconds);
    }

    public void stop() {
        spinnyServo.setPower(0);
    }

    public void openFlap() {
        flapServo.setPosition(flapOpen);
    }

    public void openFlapForDuration(double durationSeconds) {
        openFlap();
    }

    public void closeFlap() {
        flapServo.setPosition(flapClosed);
    }

}
