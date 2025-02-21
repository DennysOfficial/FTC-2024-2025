package org.firstinspires.ftc.teamcode.RobotStuff.individual_components;

import android.util.Range;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.MathStuff;

@Config
public class RightPivot extends ControlAxis {
    RightLift rightLift;

    public void assignLift(RightLift rightLift) {
        if (rightLift == null)
            throw new NullPointerException("the lift you tried to assign is null you goober");
        this.rightLift = rightLift;
    }

    @Override
    float getInput() {
        return (config.inputMap == null) ? 0 : (float) config.inputMap.getPivotStick();
    }

    static final int encoderCountsPerRevMotor = 28;
    static final double finalGearRatio = (28.0 / 150.0) * (1. / 5.23) * (1. / 5.23); // rotations of final over rotations of motor
    static final double encoderCountsPerRevFinal = encoderCountsPerRevMotor / finalGearRatio;
    static final double encoderCountsPerDeg = encoderCountsPerRevFinal / 360.0;

    static final double extendedLiftPosition = 30;
    public static double extendedGComp = 0.2;
    public static double retractedGComp = 0.05;

    public static double gCompAngleOffset = 0;


    public static Range<Double> softLimits;


    @Override
    float getVelocityControlMaxRate() {
        return config.sensitivities.getPivotRate();
    }

    @Override
    float getTorqueControlSensitivity() {
        return config.sensitivities.getPivotSensitivity();
    }

    @Override
    protected void addMotors() {
        motors.addMotor(config.deviceConfig.rightPivot, DcMotorSimple.Direction.FORWARD);

        motors.setTargetPosition(0);
        motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public RightPivot(ControlMode defaultControlMode, OpMode opMode, RobotConfig config) {
        super(defaultControlMode, opMode, config, "RightPivot", "Degrees", 1.0 / encoderCountsPerDeg);

        softLimits = new Range<>(-69.0, 97.0);
    }

    double previousRightLiftTargetPosition = Double.NaN;

    public void setTargetPosition(double targetPosition) {
        if (rightLift == null)
            throw new NullPointerException("run the assign lift method before setting target position");

        if (targetPosition == getTargetPosition() && previousRightLiftTargetPosition == (previousRightLiftTargetPosition = rightLift.getTargetPosition()))
            return;

        double dynamicLowerLimit = -1 * Math.asin(config.getRearExtensionLimitInch() / (rightLift.retractedExtension + rightLift.getTargetPosition()));
        dynamicLowerLimit = Math.toDegrees(dynamicLowerLimit);
        targetPosition = MathUtils.clamp(targetPosition, dynamicLowerLimit, Double.POSITIVE_INFINITY);

        //opMode.telemetry.addData("pivotDynamicLimit", dynamicLowerLimit);
        super.setTargetPosition(targetPosition);

    }


    public static double velocityFeedforwardCoefficientRetracted = 0;
    public static double KpRetracted = 0.2;
    public static double KiRetracted = 0.0;
    public static double KdRetracted = 0.005;

    public static double velocityFeedforwardCoefficientExtended = 0;
    public static double KpExtended = 0.2;
    public static double KiExtended = 0.0;
    public static double KdExtended = 0.008;

    @Override
    double getKp() {
        if (rightLift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return MathStuff.lerp(KpRetracted, KpExtended, rightLift.getPosition() / extendedLiftPosition);
    }

    @Override
    double getKi() {
        if (rightLift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return MathStuff.lerp(KiRetracted, KiExtended, rightLift.getPosition() / extendedLiftPosition);
    }


    @Override
    double getKd() {
        if (rightLift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return MathStuff.lerp(KdRetracted, KdExtended, rightLift.getPosition() / extendedLiftPosition);
    }

    @Override
    double getStaticFeedforward(double targetDirection) {
        if (rightLift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return -calculateTorqueGravity(rightLift.getPosition());
    }

    @Override
    double getVelocityFeedforward() {
        return targetVelocity * getVelocityFeedforwardCoefficient();
    }

    @Override
    double getAccelerationFeedforward() {
        return 0;
    }

    double getVelocityFeedforwardCoefficient() {
        if (rightLift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return MathStuff.lerp(velocityFeedforwardCoefficientRetracted, velocityFeedforwardCoefficientExtended, rightLift.getPosition() / extendedLiftPosition);
    }

    @Override
    void miscUpdate() {

        // unitsPerEncoderCount = 1.0 / (encoderCountsPerDeg * movementScaleMultiplier);

    }

    double calculateTorqueGravity(double liftExtension) {
        double interpolationAmount = liftExtension / extendedLiftPosition;
        //opMode.telemetry.addData("interpolation amount", interpolationAmount);

        return Math.sin(gCompAngleOffset + Math.toRadians(getPosition())) * MathStuff.lerp(retractedGComp, extendedGComp, interpolationAmount);
    }
}
