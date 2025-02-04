package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Ramp;

public class FullPIDDrive extends DriveModeBase {

    public static PIDCoefficients turnPid = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients xPID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients yPID = new PIDCoefficients(0, 0, 0);

    double drive, strafe, turn;
    SparkFunOTOS otos;
    SparkFunOTOS.Pose2D robotPose;

    Ramp xAccelerationControl;
    double xRequestedTargetVelocity;
    double xTargetVelocity;
    double xTargetPosition;

    Ramp yAccelerationControl;
    double yRequestedTargetVelocity;
    double yTargetVelocity;
    double yTargetPosition;

    Ramp hAccelerationControl;
    double hRequestedTargetVelocity;
    double hTargetVelocity;
    double hTargetPosition;


    public void telemetryPose() {
        opMode.telemetry.addData("heading %f", robotPose.h);
        opMode.telemetry.addData("x %f", robotPose.x);
        opMode.telemetry.addData("y %f", robotPose.y);
    }

    public FullPIDDrive(OpMode opMode, RobotConfig config) {
        super(opMode, config);
        otos = opMode.hardwareMap.get(SparkFunOTOS.class, "otos");

        robotPose = otos.getPosition();

        xTargetPosition = robotPose.x;
        yTargetPosition = robotPose.y;


    }

    void updateRequestedDriveVector() {
        xRequestedTargetVelocity = Math.cos(robotPose.h) * config.inputMap.getStrafeStick();
        xRequestedTargetVelocity = Math.sin(robotPose.h) * config.inputMap.getForwardStick();

        yRequestedTargetVelocity = Math.sin(robotPose.h) * config.inputMap.getStrafeStick();
        yRequestedTargetVelocity = Math.cos(robotPose.h) * config.inputMap.getForwardStick();
    }

    @Override
    public void updateDrive(double deltaTime) {
        robotPose = otos.getPosition();
        setDriveVector(drive, strafe, turn);


    }
}
