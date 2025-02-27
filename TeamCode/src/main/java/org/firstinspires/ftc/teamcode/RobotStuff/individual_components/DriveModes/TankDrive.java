package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.rowanmcalpin.nextftc.ftc.driving.DifferentialTankDriverControlled;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.Controllable;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

import kotlin.jvm.functions.Function0;

@Config
public class TankDrive extends DriveModeBase {

    IMU imu;

    MotorEx frontLeft = new MotorEx(frontLeftDrive);
    MotorEx frontRight = new MotorEx(frontRightDrive);
    MotorEx backLeft = new MotorEx(backLeftDrive);
    MotorEx backRight = new MotorEx(backRightDrive);

    MotorGroup leftMotors = new MotorGroup(frontLeft, backLeft);
    MotorGroup rightMotors = new MotorGroup(frontRight, backRight);

    DifferentialTankDriverControlled vroom = new DifferentialTankDriverControlled(leftMotors, rightMotors, config.playerOne.left_stick, config.playerOne.right_stick);


    public TankDrive(OpMode opMode, RobotConfig config) {
        super(opMode, config);
        imu = opMode.hardwareMap.get(IMU.class, "imu");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void updateDrive(double deltaTime) {
        vroom.update();
    }
}
