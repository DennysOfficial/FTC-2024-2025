package org.firstinspires.ftc.teamcode.individual_components;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.profiles_and_base_settings.Settings;

public class Pivot {

    // Deg limits are a maximum travel for 0 - 1 input range
    // pivot arm down and in line with the lift is 0deg
    //counterclockwise when looking at the robot from the right is positive
    public final double lowerLimitDeg = 45; // needs to be measured and adjusted accordingly TODone
    public final double upperLimitDeg = 360;
    public final double servoRangeDeg = upperLimitDeg - lowerLimitDeg;

    final double upperLimit = 1;
    final double lowerLimit = 0;
    final double[] digitalLimitsDeg = new double[]{0,350};

    public double misalignment = 0.03; // possibly  beneficial to leave a slight amount of misalignment to compensate for backlash in the gears
    public double currentPosition = 0.5; // starting position if using manual move

    public boolean debugModeEnabled = false;
    Servo leftServo;
    Servo rightServo;
    LinearOpMode opMode;
    Settings settings;

    public Pivot(LinearOpMode opModeTemp, Settings settingsTemp) {
        opMode = opModeTemp;
        settings = settingsTemp;

        leftServo = opMode.hardwareMap.get(Servo.class, "Pivot-0");
        rightServo = opMode.hardwareMap.get(Servo.class, "Pivot-1");
    }

    public void directControl(double deltaTime) {
        setTargetPositionDeg(getTargetPositionDeg() + settings.getPivotStick() * settings.pivotSensitivity * deltaTime * 360);

        if (debugModeEnabled) {
            opMode.telemetry.addLine();
            opMode.telemetry.addData("Pivot stick", "%4.2f", settings.getPivotStick());
        }

    }

    public void moveToPosition() {

        leftServo.setPosition(currentPosition + misalignment);
        rightServo.setPosition(1 - currentPosition);

        if (debugModeEnabled) {
            opMode.telemetry.addLine();
            //opMode.telemetry.addData("Pivot stick", "%4.2f", settings.getPivotStick());
            opMode.telemetry.addData("Pivot Angle:", "%4.2f", getTargetPositionDeg());
        }

    }

    public void setTargetPosition(double targetPosition) { // 0-1 range
        currentPosition = targetPosition;
        moveToPosition();
    }

    public double getTargetPositionDeg() {
        return currentPosition * servoRangeDeg + lowerLimitDeg;
    }

    public void setTargetPositionDeg(double targetPosition) {
        targetPosition = MathUtils.clamp(targetPosition, digitalLimitsDeg[0], digitalLimitsDeg[1]);

        currentPosition = (targetPosition - lowerLimitDeg);

        currentPosition /= servoRangeDeg;
        moveToPosition();
    }

}
