package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.SensorData;

public class ConstantTorqueMotor {
    final DcMotorEx motor;

    SensorData sensorData;

    public final double maxVelocityTPS;
    public final double nominalBatteryVoltage = 12.0;

    double getBackEMF(double velocityTPS) {
        return -(velocityTPS / maxVelocityTPS) * nominalBatteryVoltage;
    }

    double voltageDiff;
    double dutyCycle;


    public ConstantTorqueMotor(DcMotorEx motor, SensorData sensorData) {
        this.motor = motor;


        maxVelocityTPS = (motor.getMotorType().getMaxRPM() * motor.getMotorType().getTicksPerRev()) / 60.0;

        this.sensorData = sensorData;
    }

    public void setTorque(double targetTorque, double motorVelocityTPS) {

        motor.setPower(targetTorque);
//
//        voltageDiff = Math.abs(Math.copySign(targetTorque, sensorData.getBatteryVoltage()) - getBackEMF(motorVelocityTPS));
//
//        dutyCycle = targetTorque * nominalBatteryVoltage / voltageDiff;
//
//        motor.setPower(dutyCycle);
    }

}
