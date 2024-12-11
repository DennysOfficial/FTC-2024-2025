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
public class OldPivot extends ControlAxis { //schrödinger's code

    OldLift oldLift;

    public void assignLift(OldLift oldLift) {
        if (oldLift == null)
            throw new NullPointerException("the lift you tried to assign is null you goober");
        this.oldLift = oldLift;
    }

    static final int encoderCountsPerRevMotor = 28;
    static final double finalGearRatio = 1. / 200.; // rotations of final over rotations of motor
    static final double encoderCountsPerRevFinal = encoderCountsPerRevMotor / finalGearRatio;
    static final double encoderCountsPerDeg = encoderCountsPerRevFinal / 360;

    static final double extendedLiftPosition = 30;
    public static double extendedGComp = 0.2;
    public static double retractedGComp = 0.12;

    double getKp() {
        if (oldLift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return MathStuff.lerp(KpRetracted, KpExtended, oldLift.getPosition() / extendedLiftPosition);
    }

    double getKi() {
        if (oldLift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return MathStuff.lerp(KiRetracted, KiExtended, oldLift.getPosition() / extendedLiftPosition);
    }

    double getKd() {
        if (oldLift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return MathStuff.lerp(KdRetracted, KdExtended, oldLift.getPosition() / extendedLiftPosition);
    }

    @Override
    double getStaticFeedforward(double targetDirection) {
        if (oldLift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return calculateTorqueGravity(oldLift.getPosition());
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
        if (oldLift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return MathStuff.lerp(velocityFeedforwardCoefficientRetracted, velocityFeedforwardCoefficientExtended, oldLift.getPosition() / extendedLiftPosition);
    }

    public static double velocityFeedforwardCoefficientRetracted = 0;
    public static double KpRetracted = 0.1;
    public static double KiRetracted = 0.01;
    public static double KdRetracted = 0.005;

    public static double velocityFeedforwardCoefficientExtended = 0;
    public static double KpExtended = 0.1;
    public static double KiExtended = 0.01;
    public static double KdExtended = 0.005;


    @Override
    float getInput() {
        return (config.inputMap == null) ? 0 : (float) config.inputMap.getPivotStick();
    }

    @Override
    float getVelocityControlMaxRate() {
        return config.sensitivities.getPivotRate();
    }

    @Override
    float getTorqueControlSensitivity() {
        return config.sensitivities.getPivotSensitivity();
    }

    @Override
    protected void initMotors() {
        motors.addMotor(config.deviceConfig.leftPivot, DcMotorSimple.Direction.FORWARD);
        motors.addMotor(config.deviceConfig.rightPivot, DcMotorSimple.Direction.FORWARD);

        motors.setTargetPosition(0);
        //motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public OldPivot(ControlMode defaultControlMode, OpMode opMode, RobotConfig config) {
        super(defaultControlMode, opMode, config, "Pivot", "Degrees", 1.0 / encoderCountsPerDeg);

        softLimits = new Range<>(-40.0, 86.9);
    }

    double previousTargetLiftPosition = Double.NaN;

    @Override
    public void setTargetPosition(double targetPosition) {
        if (targetPosition == getTargetPosition() && previousTargetLiftPosition == (previousTargetLiftPosition = lift.getTargetPosition()))
            return;

        if (oldLift == null)
            throw new NullPointerException("run the assign lift method before setting target position");

        double dynamicLowerLimit = -1 * Math.asin(config.getRearExtensionLimitInch() / (config.getRetractedLiftLengthInch() + oldLift.getPosition()));
        dynamicLowerLimit = Math.toDegrees(dynamicLowerLimit);
        targetPosition = MathUtils.clamp(targetPosition, dynamicLowerLimit, Double.POSITIVE_INFINITY);

        //opMode.telemetry.addData("pivotDynamicLimit", dynamicLowerLimit);
        super.setTargetPosition(targetPosition);
    }

    @Override
    void miscUpdate() {

    }


    double calculateTorqueGravity(double liftExtension) {
        double interpolationAmount = liftExtension / extendedLiftPosition;
        //opMode.telemetry.addData("interpolation amount", interpolationAmount);

        return Math.sin(Math.toRadians(getPosition())) * MathStuff.lerp(retractedGComp, extendedGComp, interpolationAmount);
    }

}
