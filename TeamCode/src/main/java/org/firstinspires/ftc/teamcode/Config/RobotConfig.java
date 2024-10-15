package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Config.SubConfigs.DebugConfig;
import org.firstinspires.ftc.teamcode.Config.SubConfigs.DeviceConfig;
import org.firstinspires.ftc.teamcode.Config.SubConfigs.InputMap;
import org.firstinspires.ftc.teamcode.Config.SubConfigs.Sensitivities;

@Config
public class RobotConfig {

    LinearOpMode opMode;

    public final DeviceConfig deviceConfig = new DeviceConfig();
    public final DebugConfig debugConfig = new DebugConfig();
    public final InputMap inputMap;
    public final Sensitivities sensitivities = new Sensitivities();

    public final SensorData sensorData;


    public RobotConfig(LinearOpMode opMode) {
        this.opMode = opMode;
        inputMap = new InputMap(opMode.gamepad1, opMode.gamepad2);
        sensorData = new SensorData(opMode.hardwareMap);
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
