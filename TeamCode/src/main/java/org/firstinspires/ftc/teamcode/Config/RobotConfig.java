package org.firstinspires.ftc.teamcode.Config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Config.SubConfigs.DebugConfig;
import org.firstinspires.ftc.teamcode.Config.SubConfigs.DeviceConfig;

@Config
public class RobotConfig {

    LinearOpMode opMode;

    public DeviceConfig deviceConfig = new DeviceConfig();
    public DebugConfig debugConfig = new DebugConfig();

    public final VoltageSensor batteryVoltageSensor;

    VoltageSensor getBatteryVoltageSensor() {
        for (VoltageSensor sensor : opMode.hardwareMap.voltageSensor) {
            if (sensor.getVoltage() > 0)
                return sensor;
        }
        return null;
    }

    public RobotConfig(LinearOpMode opMode) {
        this.opMode = opMode;
        batteryVoltageSensor = getBatteryVoltageSensor();
    }

    public static float driveSensitivity = 1;

    public float getDriveSensitivity() {
        return driveSensitivity;
    }

    public static float forwardSensitivity = 1; // basic driving sensitivities only relative to each other

    public float getForwardSensitivity() {
        return forwardSensitivity;
    }

    public static float turningSensitivity = .69f;

    public float getTurningSensitivity() {
        return turningSensitivity;
    }

    public static float strafingSensitivity = 1;

    public float getStrafingSensitivity() {
        return strafingSensitivity;
    }

    public static float liftRate = 10; // inches per second

    public float getLiftRate() {
        return liftRate;
    }

    public static float liftSensitivity = 1;

    public float getLiftSensitivity() {
        return liftSensitivity;
    }

    public static float pivotRate = 50; //degrees per second

    public float getPivotRate() {
        return pivotRate;
    }

    public static float pivotSensitivity = 0.3f; //thingamabobs per yugvjkhjblk

    public float getPivotSensitivity() {
        return pivotSensitivity;
    }

    public double getForwardStick() {
        return opMode.gamepad1.left_stick_y;
    } // button mapping

    public double getStrafeStick() {
        return -1 * opMode.gamepad1.left_stick_x;
    }

    public double getTurnStick() {
        return opMode.gamepad1.right_stick_x;
    }

    public double getLiftStick() {
        return -1 * opMode.gamepad2.left_stick_y;
    }

    public double getPivotStick() {
        return -1 * opMode.gamepad2.right_stick_y;
    }

    public boolean getIntakeInButton() {
        return opMode.gamepad1.right_bumper;
    }

    public boolean getIntakeOutButton() {
        return opMode.gamepad1.left_bumper;
    }

    public boolean getPinchButton() {
        return opMode.gamepad2.right_bumper;
    }

    public boolean getIntakeButton() {
        return opMode.gamepad2.right_bumper;
    }

    public boolean getOuttakeButton() {
        return opMode.gamepad2.left_bumper;
    }

    public boolean getAbort() {
        return (opMode.gamepad1.left_bumper && opMode.gamepad1.right_bumper) || (opMode.gamepad2.left_bumper && opMode.gamepad2.right_bumper);
    }

    public static double autoAbortThreshold = 0.2; // how far the relevant stick has to be pushed from center before any active pid is aborted

    /**
     * how far the relevant stick has to be pushed from center before any active corresponding PID is aborted
     */
    public double getAutoAbortThreshold() {
        return autoAbortThreshold;
    }

    public boolean getFlapButton() {
        return opMode.gamepad2.left_trigger > 0.5;
    }
}
