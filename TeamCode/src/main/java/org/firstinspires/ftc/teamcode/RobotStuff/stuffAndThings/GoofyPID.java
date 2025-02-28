package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;


/**
 * global damping and different kp terms for each direction
 */
public class GoofyPID {

    double kPForwards = 0;
    double kPBackwards = 0;
    double kI = 0;
    double kD = 0;

    double P = 0;
    double I = 0;
    double D = 0;

    double previousActualValue = 0;
    Telemetry telemetry;
    RobotConfig config;

    public final String instanceName;


    public GoofyPID(Telemetry telemetry, RobotConfig config, String instanceName) {
        this.config = config;
        this.telemetry = telemetry;
        this.instanceName = instanceName;
    }

    public GoofyPID(Telemetry telemetry, RobotConfig config, double P, double kI, double kD, String instanceName) {
        this.config = config;
        this.telemetry = telemetry;
        this.kPForwards = P;
        this.kI = kI;
        this.kD = kD;
        this.instanceName = instanceName;
    }

    public void setCoefficients(double kPForwards, double kPBackwards, double kI, double kD) {
        this.kPForwards = kPForwards;
        this.kPBackwards = kPBackwards;
        this.kI = kI;
        this.kD = kD;
    }


    public double runPID(double targetValue, double actualValue, double deltaTime) {

        final double error = targetValue - actualValue;

        if (error > 0)
            P = kPForwards * error; // proportional
        else
            P = kPBackwards * error; // proportional

        I += kI * error * deltaTime; // integral

        D = kD * (previousActualValue - (previousActualValue = actualValue)) / deltaTime;


        if (config.debugConfig.getPIDDebug()) {
            telemetry.addData(instanceName + " " + "error", error);
            telemetry.addData(instanceName + " " + "value", actualValue);
            telemetry.addData(instanceName + " " + "target", targetValue);
            telemetry.addData(instanceName + " " + "Current PID values", "P: %4.2f, I: %4.2f, D %4.2f ", P, I, D);
        }

        return P + I + D;
    }


}
