package org.firstinspires.ftc.teamcode.motionControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;

public class CustomPID {

    public boolean debugMode = false;

    public double kP = 1;
    public double kI = 1;
    public double kD = 1;

    public double P = 0;
    public double I = 0;
    public double D = 0;

    public double previousActualPosition = 0;
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

    public double runPID(double targetPosition, double actualPosition, double deltaTime) {

        final double error = targetPosition - actualPosition;

        P = kP * error; // proportional

        I += kI * error * deltaTime; // integral

        D = kD * (previousActualPosition - actualPosition) / deltaTime;

        previousActualPosition = actualPosition;

        if(debugMode){
            opMode.telemetry.addData("error", error);
            opMode.telemetry.addData( "Current PID values", "P: %4.2f, I: %4.2f, D %4.2f ", P, I, D);
        }

        return P + I + D;
    }


}
