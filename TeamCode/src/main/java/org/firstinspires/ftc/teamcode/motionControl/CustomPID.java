package org.firstinspires.ftc.teamcode.motionControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;

public class CustomPID {

    public boolean debugEnabled = false;

    double kP = 0;
    double kI = 0;
    double kD = 0;

    double P = 0;
    double I = 0;
    double D = 0;

    double previousActualPosition = 0;
    OpMode opMode;
    RobotConfig config;

    public final String name;


    public CustomPID(OpMode opMode, RobotConfig config, String instanceName) {
        this.opMode = opMode;
        this.config = config;
        name = instanceName;
    }

    public CustomPID(OpMode opMode, RobotConfig config, double P, double kI, double kD, String instanceName) {
        this.opMode = opMode;
        this.config = config;
        this.kP = P;
        this.kI = kI;
        this.kD = kD;
        name = instanceName;
    }

    public void setCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void startPID(double actualPosition) {
        previousActualPosition = actualPosition;
    }

    public void setPreviousActualPosition(double previousActualPosition) {
        this.previousActualPosition = previousActualPosition;
    }

    public double runPID(double targetPosition, double actualPosition, double deltaTime) {

        return runPID(targetPosition, actualPosition, deltaTime, (previousActualPosition - (previousActualPosition = actualPosition)) / deltaTime);
    }

    public double runPID(double targetValue, double actualValue, double deltaTime, double dValueDT) {

        final double error = targetValue - actualValue;

        P = kP * error; // proportional

        I += kI * error * deltaTime; // integral

        D = kD * dValueDT;


        if (config.debugConfig.getPIDDebug()) {
            opMode.telemetry.addData(name + " " + "error", error);
            opMode.telemetry.addData(name + " " + "value", actualValue);
            opMode.telemetry.addData(name + " " + "target", targetValue);
            opMode.telemetry.addData(name + " " + "Current PID values", "P: %4.2f, I: %4.2f, D %4.2f ", P, I, D);
        }

        return P + I + D;
    }


}
