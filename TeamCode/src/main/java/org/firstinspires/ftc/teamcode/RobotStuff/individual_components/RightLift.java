package org.firstinspires.ftc.teamcode.RobotStuff.individual_components;

import android.util.Range;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

@Config
public class RightLift extends ControlAxis {

    RightPivot rightPivot;

    public final double retractedExtension = 10;

    public void assignPivot(RightPivot rightPivot) {
        if (rightPivot == null)
            throw new NullPointerException("the pivot you tried to assign is null you goober");
        this.rightPivot = rightPivot;
    }

    public static double gCompMultiplier = -0.05;

    public static double Kp = -3;
    public static double Ki = 0;
    public static double Kd = -0.03;

    public static double staticFrictionCoefficient = 0;
    public static double kineticFrictionCoefficient = 0;
    public static double staticThreshold = 0.1;

    @Override
    double getKp() {
        return Kp;
    }

    @Override
    double getKi() {
        return Ki;
    }

    @Override
    double getKd() {
        return Kd;
    }

    public static double velocityFeedforwardCoefficient = 0;


    @Override
    float getInput() {
        return (config.inputMap == null) ? 0 : (float) config.inputMap.getLiftStick();
    }

    @Override
    float getVelocityControlMaxRate() {
        return config.sensitivities.getLiftRate();
    }

    @Override
    float getTorqueControlSensitivity() {
        return config.sensitivities.getLiftSensitivity();
    }

    @Override
    protected void addMotors() {
        motors.addMotor(config.deviceConfig.rightLift, DcMotorSimple.Direction.FORWARD);

        motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    double getStaticFeedforward(double targetDirection) {
        if (rightPivot == null)
            throw new NullPointerException("run the assign pivot method before running anything else");

        return staticFrictionForce(targetDirection, staticFrictionCoefficient, staticThreshold) + Math.cos(Math.toRadians(rightPivot.getPosition())) * gCompMultiplier;
    }

    @Override
    double getVelocityFeedforward() {
        return kineticFrictionForce(targetVelocity, kineticFrictionCoefficient, staticThreshold) + targetVelocity * velocityFeedforwardCoefficient;
    }

    @Override
    double getAccelerationFeedforward() {
        return 0;
    }


    public RightLift(ControlMode defaultControlMode, OpMode opMode, RobotConfig config) {
        super(defaultControlMode, opMode, config, "RightLift", "inches", 38.6/4390.0);

        softLimits = new Range<>(0.5, 36.9);

        physicalLimits = new Range<>(0.0, Double.POSITIVE_INFINITY);
    }

    double previousRightPivotTargetPosition = Double.NaN;
    @Override
    public void setTargetPosition(double targetPosition) {
        if (rightPivot == null)
            throw new NullPointerException("run the assign pivot method before setting target position");

        if (targetPosition == getTargetPosition() && previousRightPivotTargetPosition == (previousRightPivotTargetPosition = rightPivot.getTargetPosition()))
            return;

        double dynamicUpperLimit = config.getFrontExtensionLimitInch() / Math.sin(Math.toRadians(rightPivot.getTargetPosition())) - retractedExtension;
        dynamicUpperLimit = Math.abs(dynamicUpperLimit);
        targetPosition = MathUtils.clamp(targetPosition, Double.NEGATIVE_INFINITY, dynamicUpperLimit);

        //opMode.telemetry.addData("liftDynamicLimit", upperLimit);

        super.setTargetPosition(targetPosition);
    }

    @Override
    void miscUpdate() {

    }

}
