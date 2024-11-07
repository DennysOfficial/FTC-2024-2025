package org.firstinspires.ftc.teamcode.motionControl;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Config.SensorData;

import java.util.ArrayList;
import java.util.List;

public class MultiTorqueMotor extends MultiMotor {

    SensorData sensorData;

    List<ConstantTorqueMotor> torqueMotors;

    public MultiTorqueMotor(HardwareMap hardwareMap, SensorData sensorData) {
        super(hardwareMap);
        torqueMotors = new ArrayList<ConstantTorqueMotor>();
        this.sensorData = sensorData;
    }

    @Override
    public void addMotor(String deviceName, Direction direction) {

        motors.add(hardwareMap.get(DcMotorEx.class, deviceName));

        motors.get(motors.size() - 1).setDirection(direction);
        motors.get(motors.size() - 1).setTargetPosition(targetPosition);

        forceUpdateRunMode(runMode);

        torqueMotors.add(new ConstantTorqueMotor(motors.get(motors.size() - 1), sensorData));
    }

    public void setTorque(double torque, double velocityTPS) {
        setMode(RunMode.RUN_WITHOUT_ENCODER);
        motorPower = torque;
        for (ConstantTorqueMotor torqueMotor : torqueMotors)
            torqueMotor.setTorque(torque, velocityTPS);
    }
}
