package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

public class CustomPID {

    double kP = 0;
    double kI = 0;
    double kD = 0;

    double P = 0;
    double I = 0;
    double D = 0;

    double previousActualPosition = Double.NaN;
    Telemetry telemetry;
    RobotConfig config;

    public final String instanceName;


    public CustomPID(Telemetry telemetry, RobotConfig config, String instanceName) {
        this.config = config;
        this.telemetry = telemetry;
        this.instanceName = instanceName;
    }

    public CustomPID(Telemetry telemetry, RobotConfig config, double P, double kI, double kD, String instanceName) {
        this.config = config;
        this.telemetry = telemetry;
        this.kP = P;
        this.kI = kI;
        this.kD = kD;
        this.instanceName = instanceName;
    }

    public void setCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }


    public double runPID(double targetValue, double actualValue, double deltaTime) {

        if (Double.isNaN(previousActualPosition)) {
            previousActualPosition = actualValue;
            return 0;
        }

        return runPID(targetValue, actualValue, deltaTime, (previousActualPosition - (previousActualPosition = actualValue)) / deltaTime);
    }

    public double runPID(double targetValue, double actualValue, double deltaTime, double dValueDT) {

        if (Double.isNaN(previousActualPosition)) {
            previousActualPosition = actualValue;
            return 0;
        }

        final double error = targetValue - actualValue;

        P = kP * error; // proportional

        I += kI * error * deltaTime; // integral

        D = kD * dValueDT;


        if (config.debugConfig.getPIDDebug()) {
            telemetry.addData(instanceName + " " + "error", error);
            telemetry.addData(instanceName + " " + "value", actualValue);
            telemetry.addData(instanceName + " " + "target", targetValue);
            telemetry.addData(instanceName + " " + "Current PID values", "P: %4.2f, I: %4.2f, D %4.2f ", P, I, D);
        }

        return P + I + D;
    }


}
