package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.CustomPID;

@Config
public class HoldHeading extends DriveModeBase {

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    double targetRad;

    CustomPID HeadingPID;
    IMU imu;

    double[] motorPowers = new double[4];

    double targetHeading;

    public HoldHeading(OpMode opMode, RobotConfig config) {
        super(opMode, config);
        HeadingPID = new CustomPID(opMode.telemetry, config, "HeadingPID");
        imu = opMode.hardwareMap.get(IMU.class, "imu");
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
    public void updateDrive(double deltaTime) { //TODO: USE NEXTFTC MECANUMDRIVERCONTROLLED WHEN THIS WORKS

        telemetryAngleVelocity();

        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);

        double yawVelocity = angularVelocity.zRotationRate;

        HeadingPID.setCoefficients(kP, kI, kD);

        double strafe = config.playerOne.strafeAxis.getValue() * config.sensitivities.getStrafingSensitivity();
        double drive = config.playerOne.forwardAxis.getValue() * config.sensitivities.getForwardSensitivity();
        double normTurn = config.playerOne.turnAxis.getValue() * config.sensitivities.getTurningSensitivity();
        double PIDturn = HeadingPID.lockYaw(targetRad, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), deltaTime);
        double turn = 0;
        targetRad = Math.toRadians(targetHeading);

        if (config.playerOne.turnAxis.getState()) {
            turn = normTurn;
            updateHeading();
        } else if (yawVelocity < 0.1 & yawVelocity > -0.1){
            turn = PIDturn;
        }


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

    public void updateHeading() {
        targetHeading = getHeadingDeg();
    }

}
