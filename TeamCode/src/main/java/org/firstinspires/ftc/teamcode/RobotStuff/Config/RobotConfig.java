package org.firstinspires.ftc.teamcode.RobotStuff.Config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.SubConfigs.DebugConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.SubConfigs.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.SubConfigs.InputMap;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.SubConfigs.Sensitivities;

@Config
public class RobotConfig {
    public static double rearExtensionLimitInch = 13;

    public double getRearExtensionLimitInch() {
        return rearExtensionLimitInch;
    }

    public static double frontExtensionLimitInch = 20;

    public double getFrontExtensionLimitInch() {
        return frontExtensionLimitInch;
    }

    OpMode opMode;

    public final DeviceConfig deviceConfig = new DeviceConfig();
    public final DebugConfig debugConfig = new DebugConfig();

    public final InputMap inputMap;
    public final Sensitivities sensitivities = new Sensitivities();

    public final SensorData sensorData;


    public RobotConfig(OpMode opMode) {
        this.opMode = opMode;
        inputMap = new InputMap(opMode.gamepad1, opMode.gamepad2);
        sensorData = new SensorData(opMode.hardwareMap);
    }

    public RobotConfig(HardwareMap hardwareMap) {
        this.opMode = null;
        inputMap = null;
        sensorData = new SensorData(hardwareMap);
    }


    /**
     * how far the relevant stick has to be pushed from center before any active corresponding PID is aborted
     */
    public static double autoAbortThreshold = 0.2; // how far the relevant stick has to be pushed from center before any active pid is aborted

    /**
     * returns how far the relevant stick has to be pushed from center before any active corresponding PID is aborted
     */
    public double getAutoAbortThreshold() {
        return autoAbortThreshold;
    }


}
