package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Ramp;

public class FullPIDDrive extends DriveModeBase{

    double drive, strafe, turn;
    SparkFunOTOS otos;

    Ramp xAccelerationControl;
    double xRequestedTargetVelocity;
    double xTargetVelocity;
    Ramp yAccelerationControl;
    double yRequestedTargetVelocity;
    double yTargetVelocity;



    public FullPIDDrive(OpMode opMode, RobotConfig config) {
        super(opMode, config);

    }

    @Override
    public void updateDrive(double deltaTime) {
        setDriveVector(drive,strafe,turn);
    }
}
