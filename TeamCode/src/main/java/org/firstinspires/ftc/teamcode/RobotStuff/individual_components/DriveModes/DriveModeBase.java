package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.fancyMotorThings.MultiMotor;

public abstract class DriveModeBase {

    protected OpMode opMode;

    protected RobotConfig config;
    protected DcMotorEx frontLeftDrive;
    protected DcMotorEx backLeftDrive;
    protected DcMotorEx frontRightDrive;
    protected DcMotorEx backRightDrive;

    protected MultiMotor motors;

    public DriveModeBase(OpMode opMode, RobotConfig config) {
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

    private double[] motorPowers = new double[4];
    protected void setDriveVector(double drive, double strafe, double turn){
        motorPowers[0] = drive + strafe + turn;    // Front Left
        motorPowers[1] = drive - strafe - turn;    // front right
        motorPowers[2] = drive - strafe + turn;    // back left
        motorPowers[3] = drive + strafe - turn;

        clampVectorMagnitude(motorPowers);

        frontLeftDrive.setPower(motorPowers[0] * config.sensitivities.getDriveSensitivity());
        frontRightDrive.setPower(motorPowers[1] * config.sensitivities.getDriveSensitivity());
        backLeftDrive.setPower(motorPowers[2] * config.sensitivities.getDriveSensitivity());
        backRightDrive.setPower(motorPowers[3] * config.sensitivities.getDriveSensitivity());
    }
    void clampVectorMagnitude(double[] inputArray) {
        double max;
        max = Math.max(Math.abs(inputArray[0]), Math.abs(inputArray[1]));
        max = Math.max(max, Math.abs(inputArray[2]));
        max = Math.max(max, Math.abs(inputArray[3]));

        if (max > 1)
            for (int i = 0; i < inputArray.length; i++)
                inputArray[i] /= max;
    }

    abstract public void updateDrive(double deltaTime);

    abstract public void resetDrive();
}
