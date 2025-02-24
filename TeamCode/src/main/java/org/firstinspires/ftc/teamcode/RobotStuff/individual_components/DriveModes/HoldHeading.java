package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.CustomPID;

@Config
public class HoldHeading extends DriveModeBase {

    public static double turnFeedforwardCoefficient = 0.02;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    double sumIntegral = 0;
    private double lastError = 0;

    CustomPID HeadingPID;
    IMU imu;

    double[] motorPowers = new double[4];

    double targetHeading;

    ElapsedTime timer = new ElapsedTime();

    public HoldHeading(OpMode opMode, RobotConfig config) {
        super(opMode, config);
        HeadingPID = new CustomPID(opMode.telemetry, config, "HeadingPID");
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


        double strafe = config.playerOne.strafeAxis.getValue() * config.sensitivities.getStrafingSensitivity();
        double drive = config.playerOne.forwardAxis.getValue() * config.sensitivities.getForwardSensitivity();
        double turn = config.playerOne.turnAxis.getValue() * config.sensitivities.getTurningSensitivity();

        if (!config.playerOne.turnAxis.getState()) {//config.playerOne.turnAxis.getState() returns true if the joystick is being moved- this inverts it to false, so the
            double targetRad = Math.toRadians(getHeadingDeg());//code only runs when the joystick is not moved- otherwise the robot would autocorrect even if we want to turn
            turn = HeadingPID.lockYaw(targetRad, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS), deltaTime);
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


}
