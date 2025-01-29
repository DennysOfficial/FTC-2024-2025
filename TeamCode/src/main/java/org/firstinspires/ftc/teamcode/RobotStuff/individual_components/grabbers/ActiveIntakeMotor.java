package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Timer;
@Config
public class ActiveIntakeMotor {

    OpMode opMode;
    RobotConfig config;


    DcMotor spinnyMotor;

    Servo flapServo;

    Servo wristServo;

    public static double wristUp = 0.1;
    public static double wristDown = 0.1;

    static float intakeSpeed = -1;
    static float outtakeSpeed = 1;

    static double flapOpen = 0;
    static double flapClosed = 0.9;




    public ActiveIntakeMotor(OpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;
        spinnyMotor = opMode.hardwareMap.get(DcMotor.class, config.deviceConfig.intakeMotor);
        flapServo = opMode.hardwareMap.get(Servo.class, config.deviceConfig.flapServo);
        wristServo = opMode.hardwareMap.get(Servo.class, config.deviceConfig.wristServo);
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
        //flapControl();
        if(config.inputMap.getIntakeWristDown())
            wristServo.setPosition(wristDown);
        else
            wristServo.setPosition(wristUp);
    }

    public void wheelControl() {
        spinnyMotor.setPower(0);

        if (config.inputMap.getIntakeButton())
            spinnyMotor.setPower(intakeSpeed);

        else if (config.inputMap.getOuttakeButton())
            spinnyMotor.setPower(outtakeSpeed);
    }

    public void flapControl() {

        if (config.inputMap.getFlapButton()) {
            flapServo.setPosition(flapOpen);
            return;
        }

        flapServo.setPosition(flapClosed);
    }

    public void intake() {
        spinnyMotor.setPower(intakeSpeed);
    }


    public void intakeForDuration(double durationSeconds) {
        intake();
        stopTimer = new Timer(durationSeconds);
    }

    public void outtake() {
        spinnyMotor.setPower(outtakeSpeed);
    }

    public void outtakeForDuration(double durationSeconds) {
        outtake();
        stopTimer = new Timer(durationSeconds);
    }

    public void stop() {
        spinnyMotor.setPower(0);
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
