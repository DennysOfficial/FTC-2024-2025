package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

public class CustomPID {

    double kP = 0;
    double kI = 0;
    double kD = 0;

    double P = 0;
    double I = 0;
    double D = 0;

    double lastError = 0;
    double integralSum = 0;

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

    public double lockYaw(double targetPos, double currentPos, double deltaTime) {
        double error = angleWrap(targetPos - currentPos);
        integralSum += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;
        lastError = error;
        P = (error * kP);
        I = (integralSum * kI);
        D = (derivative * kD);
        yawTelemetry(error, derivative, targetPos, currentPos, lastError);
        return P + I + D;
    }


    public double runPID(double targetPosition, double actualPosition, double deltaTime) {

        if (Double.isNaN(previousActualPosition)) {
            previousActualPosition = actualPosition;
            return 0;
        }

        return runPID(targetPosition, actualPosition, deltaTime, (previousActualPosition - (previousActualPosition = actualPosition)) / deltaTime);
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

    public double angleWrap(double radians) { // so robot doesn't rotate 350deg to get from 5deg to 355deg

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }

    public void yawTelemetry(double error, double derivative, double targetPos, double currentPos, double lastError) {
        telemetry.addData("error:", error);
        telemetry.addData("target radians:", targetPos);
        telemetry.addData("current radians:", currentPos);
        telemetry.addData("derivative:", derivative);
        telemetry.addData("last error:", lastError);
        telemetry.addData("P:", P);
        telemetry.addData("I:", I);
        telemetry.addData("D:", D);
    }

}
