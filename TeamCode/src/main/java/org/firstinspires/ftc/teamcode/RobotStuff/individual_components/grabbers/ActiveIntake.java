package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

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
        flapServo.setPosition(flapClosed);
        if (config.inputMap.getFlapButton())
            flapServo.setPosition(flapOpen);
    }

    public class Intake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            spinnyServo.setPower(intakeSpeed);
            return false;
        }
    }

    public Action intake() {
        return new Intake();
    }

    public class Outtake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            spinnyServo.setPower(outtakeSpeed);
            return false;
        }
    }

    public Action outtake() {
        return new Outtake();
    }


    public class Stop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            spinnyServo.setPower(0);
            return false;
        }
    }

    public Action stop() {
        return new Stop();
    }


    public class OpenFlap implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            flapServo.setPosition(flapOpen);
            return false;
        }
    }

    public Action openFlap() {
        return new OpenFlap();
    }


    public class CloseFlap implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            flapServo.setPosition(flapClosed);
            return false;
        }
    }

    public Action closeFlap() {
        return new CloseFlap();
    }
}
