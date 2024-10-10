package org.firstinspires.ftc.teamcode.motionControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;

public class CustomPID {

    public boolean debugEnabled = false;

    double kP = 1;
    double kI = 1;
    double kD = 1;

    double P = 0;
    double I = 0;
    double D = 0;

    double previousActualPosition = 0;
    LinearOpMode opMode;
    RobotConfig config;


    public CustomPID(LinearOpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;
    }

    public CustomPID(LinearOpMode opMode, RobotConfig config, double P, double kI, double kD) {
        this.opMode = opMode;
        this.config = config;
        this.kP = P;
        this.kI = kI;
        this.kD = kD;
    }

    public void setCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void startPID(double actualPosition) {
        previousActualPosition = actualPosition;
    }

    public void setPreviousActualPosition(double previousActualPosition){
        this.previousActualPosition = previousActualPosition;
    }

    public double runPID(double targetPosition, double actualPosition, double deltaTime) {

        return runPID(targetPosition, actualPosition, deltaTime, (previousActualPosition - (previousActualPosition = actualPosition)) / deltaTime);
    }

    public double runPID(double targetPosition, double actualPosition, double deltaTime, double velocity) {

        final double error = targetPosition - actualPosition;

        P = kP * error; // proportional

        I += kI * error * deltaTime; // integral

        D = kD * velocity;

        if (debugEnabled) {
            opMode.telemetry.addData("error", error);
            opMode.telemetry.addData("position", actualPosition);
            opMode.telemetry.addData("targetPosition", targetPosition);
            opMode.telemetry.addData("Current PID values", "P: %4.2f, I: %4.2f, D %4.2f ", P, I, D);
        }

        return P + I + D;
    }


}
