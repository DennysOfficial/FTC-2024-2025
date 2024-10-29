package org.firstinspires.ftc.teamcode.individual_components.DriveModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;

public abstract class DriveModeBase {

    protected LinearOpMode opMode;
    protected RobotConfig config;
    protected DcMotorEx frontLeftDrive;
    protected DcMotorEx backLeftDrive;
    protected DcMotorEx frontRightDrive;
    protected DcMotorEx backRightDrive;

    public DriveModeBase(LinearOpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;

        initMotors(opMode, config);
    }

    public void initMotors(LinearOpMode opMode, RobotConfig config) {

        frontLeftDrive = opMode.hardwareMap.get(DcMotorEx.class, config.deviceConfig.frontLeftDrive);
        backLeftDrive = opMode.hardwareMap.get(DcMotorEx.class, config.deviceConfig.backLeftDrive);
        backRightDrive = opMode.hardwareMap.get(DcMotorEx.class, config.deviceConfig.backRightDrive);
        frontRightDrive = opMode.hardwareMap.get(DcMotorEx.class, config.deviceConfig.frontRightDrive);

        frontLeftDrive.setDirection(config.deviceConfig.frontLeftDriveDir);
        backLeftDrive.setDirection(config.deviceConfig.backLeftDriveDir);
        backRightDrive.setDirection(config.deviceConfig.backRightDriveDir);
        frontRightDrive.setDirection(config.deviceConfig.frontRightDriveDir);
    }


    abstract public void updateDrive(double deltaTime);
}
