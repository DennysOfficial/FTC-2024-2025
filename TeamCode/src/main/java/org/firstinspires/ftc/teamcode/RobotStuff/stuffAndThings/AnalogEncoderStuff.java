package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings;

import android.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

public class AnalogEncoderStuff {

    OpMode opMode;
    RobotConfig config;
    AnalogInput analogEncoder;
    String instanceName;

    public AnalogEncoderStuff(OpMode opMode, RobotConfig config, String encoderName, String instanceName) {
        this.opMode = opMode;
        this.config = config;
        this.analogEncoder = opMode.hardwareMap.get(AnalogInput.class, encoderName);
        this.instanceName = instanceName;
    }

    double analogEncoderLastPosition;
    double analogEncoderDelta;
    public static Range<Double> analogEncoderRange = new Range<>(0.0, 3.3);
    int analogEncoderRotationCount = 0;

    public static double rotationCountThreshold = 1.15;

    double analogPosition;

    static double analogToDegrees = (analogEncoderRange.getUpper() - analogEncoderRange.getLower()) * 360 * (15.0 / 150.0);

    double analogEncoderCalculatedAngle;

    void miscUpdate() {
        analogEncoderDelta = -(analogEncoderLastPosition - (analogEncoderLastPosition = analogEncoder.getVoltage()));

        if (Math.abs(analogEncoderDelta) > rotationCountThreshold) {
            analogEncoderRotationCount += (analogEncoderDelta > 0) ? -1 : 1;
        }

        analogPosition = analogEncoderLastPosition + analogEncoderRotationCount * (analogEncoderRange.getUpper() - analogEncoderRange.getLower());

        analogEncoderCalculatedAngle = analogPosition * analogToDegrees;

        if (config.debugConfig.getAnalogEncoderDebug()) {
            opMode.telemetry.addLine();
            opMode.telemetry.addData(instanceName + " reading", analogEncoderLastPosition);
            opMode.telemetry.addData(instanceName + " count", analogEncoderRotationCount);
            opMode.telemetry.addData(instanceName + " calculated analog position", analogPosition);
            opMode.telemetry.addData(instanceName + " encoder calculated angle", analogEncoderCalculatedAngle);
        }
    }
}
