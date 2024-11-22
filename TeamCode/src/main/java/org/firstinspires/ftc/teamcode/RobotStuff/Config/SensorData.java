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
    double batteryVoltage = 12;
    public double getBatteryVoltage() {
        return batteryVoltage;
    }

}
