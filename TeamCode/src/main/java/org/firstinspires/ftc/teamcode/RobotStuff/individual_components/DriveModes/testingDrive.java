package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

public class testingDrive extends DriveModeBase {

    BNO055IMU imu;
    public testingDrive(OpMode opMode, RobotConfig config) {
        super(opMode, config);

        imu = opMode.hardwareMap.get(BNO055IMU.class,"imu");
    }



    @Override
    public void updateDrive(double deltaTime) {
        telemetryAngularVelocity();
        telemetryAccelerations();
    }

    public void telemetryAngularVelocity(){
        opMode.telemetry.addData("xAngularVelocityDPS",imu.getAngularVelocity().toAngleUnit(AngleUnit.DEGREES).xRotationRate);
        opMode.telemetry.addData("yAngularVelocityDPS",imu.getAngularVelocity().toAngleUnit(AngleUnit.DEGREES).yRotationRate);
        opMode.telemetry.addData("zAngularVelocityDPS",imu.getAngularVelocity().toAngleUnit(AngleUnit.DEGREES).zRotationRate);
    }

    public void telemetryAccelerations(){
        opMode.telemetry.addData("xAcceleration",imu.getAcceleration().xAccel);
        opMode.telemetry.addData("yAcceleration",imu.getAcceleration().yAccel);
        opMode.telemetry.addData("zAcceleration",imu.getAcceleration().zAccel);
    }


}
