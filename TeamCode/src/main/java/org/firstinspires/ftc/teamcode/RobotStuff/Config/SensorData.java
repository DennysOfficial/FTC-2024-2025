package org.firstinspires.ftc.teamcode.RobotStuff.Config;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class SensorData {

    HardwareMap hardwareMap;

    public SensorData(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        update();
    }

    public void update() {
        updateBatteryVoltage();
    }

    void updateBatteryVoltage() {
        if (batteryVoltageSensor == null) {
            batteryVoltageSensor = getBatteryVoltageSensor();
        }
        batteryVoltage = batteryVoltageSensor.getVoltage();
    }

    VoltageSensor getBatteryVoltageSensor() {
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            if (sensor.getVoltage() > 0)
                return sensor;
        }
        return null;
    }

    VoltageSensor batteryVoltageSensor;
    double batteryVoltage;

    public double getBatteryVoltage() {
        return batteryVoltage;
    }

    public double getForwardAcceleration() {
        return 0;
    }//TODO

    public double getRightAcceleration() {
        return 0;
    }//TODO

    public double getUpAcceleration() {
        return 0;
    }//TODO

    public double getYawRate() {
        return 0;
    }//TODO

    public double getPitchRate() {
        return 0;
    }//TODO

    public double getRollRate() {
        return 0;
    }//TODO


    public double getYaw() {
        return 0;
    }//TODO

    public double getPitch() {
        return 0;
    }//TODO

    public double getRoll() {
        return 0;
    }//TODO


}
