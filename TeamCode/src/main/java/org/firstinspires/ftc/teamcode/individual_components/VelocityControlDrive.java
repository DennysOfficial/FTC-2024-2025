package org.firstinspires.ftc.teamcode.individual_components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;

public class VelocityControlDrive {


    public PIDFCoefficients pidfCoefficients = new PIDFCoefficients(20, 0.1, 1, 1); // TODO needs to be adjusted to robot
    LinearOpMode opMode;
    RobotConfig config;
    DcMotorEx frontLeftDrive;
    DcMotorEx backLeftDrive;
    DcMotorEx frontRightDrive;
    DcMotorEx backRightDrive;
    double[] motorPowers = new double[4];

    public VelocityControlDrive(LinearOpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;


        frontLeftDrive = opMode.hardwareMap.get(DcMotorEx.class, "FL");
        backLeftDrive = opMode.hardwareMap.get(DcMotorEx.class, "BL");
        backRightDrive = opMode.hardwareMap.get(DcMotorEx.class, "BR");
        frontRightDrive = opMode.hardwareMap.get(DcMotorEx.class, "FR");

        //frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        //backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        //frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        //backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        backLeftDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        frontRightDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        backRightDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients); //motor go vroom

    }

    public void updatePIDCoefficients() {
        frontLeftDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        backLeftDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        frontRightDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        backRightDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void updateWheels() {
        double forwardBackward = -1 * config.getForwardStick() * config.getForwardSensitivity();  //Note: pushing stick forward gives negative value
        double strafe = -1 * config.getStrafeStick() * config.getStrafingSensitivity();
        double yaw = -1 * config.getTurnStick() * config.getTurningSensitivity();

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        motorPowers[0] = forwardBackward + strafe + yaw;    // Front Left
        motorPowers[1] = forwardBackward - strafe - yaw;    // front right
        motorPowers[2] = forwardBackward - strafe + yaw;    // back left
        motorPowers[3] = forwardBackward + strafe - yaw;    // back right

        motorPowers = normalizeArray(motorPowers);


        // Send calculated power to wheels
        frontLeftDrive.setVelocity(motorPowers[0] * 2000 * config.getDriveSensitivity());
        frontRightDrive.setVelocity(motorPowers[1] * 2000 * config.getDriveSensitivity());
        backLeftDrive.setVelocity(motorPowers[2] * 2000 * config.getDriveSensitivity());
        backRightDrive.setVelocity(motorPowers[3] * 2000 * config.getDriveSensitivity());

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

        if (max > 1.0)
            for (int i = 0; i < inputArray.length; i++)
                inputArray[i] /= max;

        return inputArray;
    }
}
