package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.CustomPID;

@Config
public class AdvancedMechanumDrive extends DriveModeBase {

    public static double rotate = 0.02;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0; // the robot has a k/d ratio of zero

    CustomPID lockYawPID;
    IMU imu;

    double targetYaw;

    double[] motorPowers = new double[4];

    public AdvancedMechanumDrive(OpMode opMode, RobotConfig config) {
        super(opMode, config);
        lockYawPID = new CustomPID(opMode.telemetry, config, "LockYaw");
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        targetYaw = getHeadingDeg();
        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    double getHeadingDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void telemetryAngleVelocity() {
        opMode.telemetry.addData("currentHeading", getHeadingDeg());
        opMode.telemetry.addData("targetHeading", targetYaw);
        opMode.telemetry.addData("pitchAngleVel", imu.getRobotAngularVelocity(AngleUnit.DEGREES).xRotationRate);
        opMode.telemetry.addData("rollAngleVel", imu.getRobotAngularVelocity(AngleUnit.DEGREES).yRotationRate);
        opMode.telemetry.addData("yawAngleVel", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
    }

    @Override
    public void updateDrive(double deltaTime) {

        double SensitivityModifier = config.sensitivities.getDriveSensitivity();


        if (config.inputMap.getSlowDown()){SensitivityModifier = config.sensitivities.getSlowDownModifier();}


        double forwardBackward = -1 * config.inputMap.getForwardStick() * config.sensitivities.getForwardSensitivity() * SensitivityModifier;  //Note: pushing stick forward gives negative value
        double strafe = -1 * config.inputMap.getStrafeStick() * config.sensitivities.getStrafingSensitivity() * SensitivityModifier;

        double targetTurnRate = -1 * config.inputMap.getTurnStick() * config.sensitivities.getTurningRateDPS();

        lockYawPID.setCoefficients(Kp, Ki, Kd);

        double yaw = rotate * targetTurnRate;

        yaw += lockYawPID.runPID(targetYaw, getHeadingDeg(), deltaTime);

        motorPowers[0] = forwardBackward + strafe + yaw;    // Front Left
        motorPowers[1] = forwardBackward - strafe - yaw;    // front right
        motorPowers[2] = forwardBackward - strafe + yaw;    // back left
        motorPowers[3] = forwardBackward + strafe - yaw;    // back right

        motorPowers = normalizeArray(motorPowers);

        frontLeftDrive.setPower(motorPowers[0] * config.sensitivities.getDriveSensitivity());
        frontRightDrive.setPower(motorPowers[1] * config.sensitivities.getDriveSensitivity());
        backLeftDrive.setPower(motorPowers[2] * config.sensitivities.getDriveSensitivity());
        backRightDrive.setPower(motorPowers[3] * config.sensitivities.getDriveSensitivity());

    }

    public boolean adamIsShort() {
        return true;
    }

    double[] normalizeArray(double[] inputArray) {

        double max;
        max = Math.max(Math.abs(inputArray[0]), Math.abs(inputArray[1]));
        max = Math.max(max, Math.abs(inputArray[2]));
        max = Math.max(max, Math.abs(inputArray[3]));

        if (max > 1)
            for (int i = 0; i < inputArray.length; i++)
                inputArray[i] /= max;

        return inputArray;
    }

}
