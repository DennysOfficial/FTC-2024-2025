package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.CustomPID;

@Config
public class HeadingPIDSteerTest extends DriveModeBase {

    public static double turnFeedforwardCoefficient = 0.02;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;

    CustomPID steeringPID;
    IMU imu;

    double[] motorPowers = new double[4];

    double targetHeading;

    public HeadingPIDSteerTest(OpMode opMode, RobotConfig config) {
        super(opMode, config);
        steeringPID = new CustomPID(opMode.telemetry, config, "steeringPID");
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        targetHeading = getHeadingDeg();
    }

    double getHeadingDeg() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void telemetryAngleVelocity() {
        opMode.telemetry.addData("Heading", getHeadingDeg());
        opMode.telemetry.addData("TargetHeading", targetHeading);
        opMode.telemetry.addData("angleVelX", imu.getRobotAngularVelocity(AngleUnit.DEGREES).xRotationRate);
        opMode.telemetry.addData("angleVelY", imu.getRobotAngularVelocity(AngleUnit.DEGREES).yRotationRate);
        opMode.telemetry.addData("angleVelZ", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
    }

    @Override
    public void updateDrive(double deltaTime) {

        telemetryAngleVelocity();

        double strafe = -1 * config.playerOne.strafeAxis.getValue() * config.sensitivities.getStrafingSensitivity();
        double drive = -1 * config.playerOne.forwardAxis.getValue() * config.sensitivities.getForwardSensitivity();

        double turn = getTurn(deltaTime);

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

    double getTurn(double deltaTime) {
        if (config.playerOne.turnAxis.getState()) { // if joystick is displaced (past the threshold)
            return config.playerOne.turnAxis.getValue() * config.sensitivities.getTurningSensitivity();
        } else { // if joystick is not being used
            double targetTurnRate = -1 * config.playerOne.turnAxis.getValue() * config.sensitivities.getTurningRateDPS();

            steeringPID.setCoefficients(Kp, Ki, Kd);

            double turn = turnFeedforwardCoefficient * targetTurnRate;

            turn += steeringPID.runPID(angleWrap(), getHeadingDeg(), deltaTime);

            return turn;
        }
    }


    public double angleWrap() {
        return Math.toDegrees(angleWrap(Math.toRadians(targetHeading - getHeadingDeg())));
    }

    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }

}
