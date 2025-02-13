package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.CustomPID;

@Config
public class HeadingPIDSteerTest extends DriveModeBase {

    public static double turnFeedforwardCoefficient = 0.02;
    public static double angleKp = 0;
    public static double angleKi = 0;
    public static double angleKd = 0;

    public static double maxAngularAcceleration = 10;


    CustomPID steeringAnglePID;
    IMU imu;

    double[] motorPowers = new double[4];

    double targetHeading;

    public HeadingPIDSteerTest(OpMode opMode, RobotConfig config) {
        super(opMode, config);
        steeringAnglePID = new CustomPID(opMode.telemetry, config, "steeringAnglePID");
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        targetHeading = getHeadingDeg();
    }

    double previousHeading;
    int rotationCount = 0;

    double getHeadingDeg() {
        double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (previousHeading * imuHeading <= 1) {
            if (previousHeading - imuHeading > 100)
                rotationCount++;
            else if (previousHeading - imuHeading < -100)
                rotationCount--;
        }

        return imuHeading + rotationCount * 360;
    }

    double getHeadingRate() {
        return imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
    }

    public void telemetryAngleVelocity() {
        opMode.telemetry.addData("Heading", getHeadingDeg());
        opMode.telemetry.addData("TargetHeading", targetHeading);
        opMode.telemetry.addData("angleVelX", imu.getRobotAngularVelocity(AngleUnit.DEGREES).xRotationRate);
        opMode.telemetry.addData("angleVelY", imu.getRobotAngularVelocity(AngleUnit.DEGREES).yRotationRate);
        opMode.telemetry.addData("angleVelZ", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
    }

    double calculateTurnRate(double requestedTargetTurnRate, double deltaTime) {
        if(requestedTargetTurnRate == previousTargetTurnRate)
            return requestedTargetTurnRate;

        double accelerationDirection = Math.copySign(1, requestedTargetTurnRate - previousTargetTurnRate);



        double targetTurnRate = accelerationDirection * previousTargetTurnRate * maxAngularAcceleration * deltaTime;

        if(Math.copySign(1, targetTurnRate - previousTargetTurnRate) != accelerationDirection)
            return requestedTargetTurnRate;

        return targetTurnRate;
    }

    double previousTargetTurnRate = 0;

    @Override
    public void updateDrive(double deltaTime) {

        telemetryAngleVelocity();

        double strafe = -1 * config.inputMap.getStrafeStick() * config.sensitivities.getStrafingSensitivity();
        double drive = -1 * config.inputMap.getForwardStick() * config.sensitivities.getForwardSensitivity();

        double requestedTargetTurnRate = -1 * config.inputMap.getTurnStick() * config.sensitivities.getTurningRateDPS();

        double targetTurnRate = calculateTurnRate(requestedTargetTurnRate, deltaTime);

        targetHeading += targetTurnRate * deltaTime;

        steeringAnglePID.setCoefficients(angleKp, angleKi, angleKd);


        double turn = turnFeedforwardCoefficient * targetTurnRate;

        turn += steeringAnglePID.runPID(targetHeading, getHeadingDeg(), deltaTime);

        turn *= -1;

        motorPowers[0] = drive + strafe + turn;    // Front Left
        motorPowers[1] = drive - strafe - turn;    // front right
        motorPowers[2] = drive - strafe + turn;    // back left
        motorPowers[3] = drive + strafe - turn;    // back right

        motorPowers = normalizeArrayKinda(motorPowers);


        // Send calculated power to wheels
        frontLeftDrive.setPower(motorPowers[0] * config.sensitivities.getDriveSensitivity());
        frontRightDrive.setPower(motorPowers[1] * config.sensitivities.getDriveSensitivity());
        backLeftDrive.setPower(motorPowers[2] * config.sensitivities.getDriveSensitivity());
        backRightDrive.setPower(motorPowers[3] * config.sensitivities.getDriveSensitivity());
    }

    double[] normalizeArrayKinda(double[] inputArray) {

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
