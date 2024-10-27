package org.firstinspires.ftc.teamcode.motionControl;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Config.SensorData;

public class ConstantTorqueMotor {
    final DcMotorEx motor;

    SensorData sensorData;

    public final double maxVelocityTPS;
    public final double nominalBatteryVoltage = 12.0;

    double outputPower;


    public ConstantTorqueMotor(DcMotorEx motor, SensorData sensorData) {
        this.motor = motor;


        maxVelocityTPS = (motor.getMotorType().getMaxRPM() * motor.getMotorType().getTicksPerRev()) / 60.0;

        this.sensorData = sensorData;
    }

    public void setTorque(double targetTorque, double motorVelocityTPS) {

        outputPower = targetTorque +   (motorVelocityTPS / maxVelocityTPS);

        outputPower /= sensorData.getBatteryVoltage() / nominalBatteryVoltage;

        motor.setPower(outputPower);
    }

}
