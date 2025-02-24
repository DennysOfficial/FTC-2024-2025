package org.firstinspires.ftc.teamcode.RobotStuff.Config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.SubConfigs.DebugConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.SubConfigs.DeviceConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.SubConfigs.InputMap;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.SubConfigs.playerOne;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.SubConfigs.playerTwo;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.SubConfigs.Sensitivities;

@Config
public class RobotConfig {

    public static double retractedLiftLengthInch = 15;

    public double getRetractedLiftLengthInch() {
        return retractedLiftLengthInch;
    }

    public static double rearExtensionLimitInch = 12;

    public double getRearExtensionLimitInch() {
        return rearExtensionLimitInch;
    }

    public static double frontExtensionLimitInch = 25 ;

    public double getFrontExtensionLimitInch() {
        return frontExtensionLimitInch;
    }

    OpMode opMode;

    public final DeviceConfig deviceConfig = new DeviceConfig();
    public final DebugConfig debugConfig = new DebugConfig();

    public final playerOne playerOne;
    public final playerTwo playerTwo;

    public final Sensitivities sensitivities = new Sensitivities();

    public final SensorData sensorData;


    public RobotConfig(OpMode opMode) {
        this.opMode = opMode;
        playerOne = new playerOne(opMode.gamepad1);
        playerTwo = new playerTwo(opMode.gamepad2);
        sensorData = new SensorData(opMode.hardwareMap);
    }

    public RobotConfig(HardwareMap hardwareMap) {
        this.playerOne = null;
        this.playerTwo = null;
        this.opMode = null;
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
