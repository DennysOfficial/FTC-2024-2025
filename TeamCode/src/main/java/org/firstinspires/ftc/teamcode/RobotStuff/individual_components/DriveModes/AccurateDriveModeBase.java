package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.fancyMotorThings.MultiMotor;

public abstract class AccurateDriveModeBase {

    protected OpMode opMode;

    protected RobotConfig config;
    protected DcMotorEx frontLeftDrive;
    protected DcMotorEx backLeftDrive;
    protected DcMotorEx frontRightDrive;
    protected DcMotorEx backRightDrive;

    protected MultiMotor motors;

    public AccurateDriveModeBase(OpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;

        initMotors(opMode, config);
    }

    public void initMotors(OpMode opMode, RobotConfig config) {

        frontLeftDrive = opMode.hardwareMap.get(DcMotorEx.class, config.deviceConfig.frontLeftDrive);
        backLeftDrive = opMode.hardwareMap.get(DcMotorEx.class, config.deviceConfig.backLeftDrive);
        backRightDrive = opMode.hardwareMap.get(DcMotorEx.class, config.deviceConfig.backRightDrive);
        frontRightDrive = opMode.hardwareMap.get(DcMotorEx.class, config.deviceConfig.frontRightDrive);

        motors = new MultiMotor(opMode.hardwareMap);
        motors.addMotor(frontLeftDrive);
        motors.addMotor(frontRightDrive);
        motors.addMotor(backLeftDrive);
        motors.addMotor(backRightDrive);

        frontLeftDrive.setDirection(config.deviceConfig.frontLeftDriveDir);
        backLeftDrive.setDirection(config.deviceConfig.backLeftDriveDir);
        backRightDrive.setDirection(config.deviceConfig.backRightDriveDir);
        frontRightDrive.setDirection(config.deviceConfig.frontRightDriveDir);
    }

    abstract public void AccurateUpdateDrive(double deltaTime, double imuYaw);

}
