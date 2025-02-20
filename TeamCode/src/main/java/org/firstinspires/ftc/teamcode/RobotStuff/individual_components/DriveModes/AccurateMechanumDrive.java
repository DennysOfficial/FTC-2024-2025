package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

@Config
public class AccurateMechanumDrive extends AccurateDriveModeBase {


    double[] motorPowers = new double[4];

    public AccurateMechanumDrive(OpMode opMode, RobotConfig config) {
        super(opMode, config);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void AccurateUpdateDrive(double deltaTime, double imuYaw) {

        double SensitivityModifier = config.sensitivities.getDriveSensitivity();

        if (config.inputMap.getSlowDown()){SensitivityModifier = config.sensitivities.getSlowDownModifier();}

        double forwardBackward = -1 * config.inputMap.getForwardStick() * config.sensitivities.getForwardSensitivity() * SensitivityModifier;  //Note: pushing stick forward gives negative value
        double strafe = -1 * config.inputMap.getStrafeStick() * config.sensitivities.getStrafingSensitivity() * SensitivityModifier;
        double yaw = fixYaw(imuYaw, SensitivityModifier);


        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        motorPowers[0] = forwardBackward + strafe + yaw;    // Front Left
        motorPowers[1] = forwardBackward - strafe - yaw;    // front right
        motorPowers[2] = forwardBackward - strafe + yaw;    // back left
        motorPowers[3] = forwardBackward + strafe - yaw;    // back right

        motorPowers = normalizeArray(motorPowers);


        // Send calculated power to wheels
        frontLeftDrive.setPower(motorPowers[0] * config.sensitivities.getDriveSensitivity());
        frontRightDrive.setPower(motorPowers[1] * config.sensitivities.getDriveSensitivity());
        backLeftDrive.setPower(motorPowers[2] * config.sensitivities.getDriveSensitivity());
        backRightDrive.setPower(motorPowers[3] * config.sensitivities.getDriveSensitivity());

        //frontRightDrive.setPower(1);

        /*
        opMode.telemetry.addLine("motor power");
        opMode.telemetry.addData("Front left/Right", "%4.2f, %4.2f", motorPowers[0], motorPowers[1]);
        opMode.telemetry.addData("Back  left/Right", "%4.2f, %4.2f", motorPowers[2], motorPowers[3]);

        opMode.telemetry.addLine();

        opMode.telemetry.addLine("target velocity");
        opMode.telemetry.addData("Front left/Right", "%4.2f, %4.2f", motorPowers[0] * 2000 * settings.driveSensitivity,motorPowers[1] * 2000 * settings.driveSensitivity);
        opMode.telemetry.addData("Back  left/Right", "%4.2f, %4.2f", motorPowers[2] * 2000 * settings.driveSensitivity, motorPowers[3] * 2000 * settings.driveSensitivity);
        opMode.telemetry.addLine();

        opMode.telemetry.addLine("actual velocity");
        opMode.telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftDrive.getVelocity(), frontRightDrive.getVelocity());
        opMode.telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftDrive.getVelocity(), backRightDrive.getVelocity());
        */

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

    public double fixYaw(double imuYaw, double SensitivityModifier) {
        if (config.inputMap.getTurnStick() == 0) {
            if ((imuYaw > 0 && imuYaw < 3 || imuYaw > 357) || (imuYaw > 87 && imuYaw < 93) || (imuYaw > 177 && imuYaw < 183) || (imuYaw > 267 && imuYaw < 273)) {
                return config.inputMap.getTurnStick() * config.sensitivities.getTurningSensitivity() * SensitivityModifier;
            } else {
                return 1 * config.sensitivities.getTurningSensitivity() * SensitivityModifier;
            }
        }
        else {return config.inputMap.getTurnStick() * config.sensitivities.getTurningSensitivity() * SensitivityModifier;}
    }
}
