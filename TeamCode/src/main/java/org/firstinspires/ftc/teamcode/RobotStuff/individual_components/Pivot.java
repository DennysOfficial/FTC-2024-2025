package org.firstinspires.ftc.teamcode.RobotStuff.individual_components;

import android.util.Range;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.MathStuff;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ReadOnlyRuntime;

@Config
public class Pivot extends ControlAxis { //schrödinger's code

    Lift lift;

    public void assignLift(Lift lift) {
        if (lift == null)
            throw new NullPointerException("the lift you tried to assign is null you goober");
        this.lift = lift;
    }

    static final int encoderCountsPerRevMotor = 28;
    static final double finalGearRatio = 1. / 200.; // rotations of final over rotations of motor
    static final double encoderCountsPerRevFinal = encoderCountsPerRevMotor / finalGearRatio;
    static final double encoderCountsPerDeg = encoderCountsPerRevFinal / 360;

    static final double extendedLiftPosition = 30;
    public static double extendedGComp = 0.2;
    public static double retractedGComp = 0.12;

    double getKp() {
        if (lift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return MathStuff.lerp(KpRetracted, KpExtended, lift.getPosition() / extendedLiftPosition);
    }

    double getKi() {
        if (lift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return MathStuff.lerp(KiRetracted, KiExtended, lift.getPosition() / extendedLiftPosition);
    }

    double getKd() {
        if (lift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return MathStuff.lerp(KdRetracted, KdExtended, lift.getPosition() / extendedLiftPosition);
    }

    @Override
    double getStaticFeedforward(double targetDirection) {
        if (lift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return calculateTorqueGravity(lift.getPosition());
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
        if (lift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return MathStuff.lerp(velocityFeedforwardCoefficientRetracted, velocityFeedforwardCoefficientExtended, lift.getPosition() / extendedLiftPosition);
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
        return (config.playerTwo == null) ? 0 : (float) config.playerTwo.pivotAxis.getValue();
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
        motors.addMotor(config.deviceConfig.leftPivot, DcMotorSimple.Direction.REVERSE);
        motors.addMotor(config.deviceConfig.rightPivot, DcMotorSimple.Direction.REVERSE);

        motors.setTargetPosition(0);
        //motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public Pivot(ControlMode defaultControlMode, OpMode opMode, RobotConfig config) {
        super(defaultControlMode, opMode, config, "Pivot", "Degrees", 1.0 / encoderCountsPerDeg);

        softLimits = new Range<>(-55.0, 86.9);
    }

    double previousTargetLiftPosition = Double.NaN;

    @Override
    public void setTargetPosition(double targetPosition) {
        if (targetPosition == getTargetPosition() && previousTargetLiftPosition == (previousTargetLiftPosition = lift.getTargetPosition()))
            return;

        if (lift == null)
            throw new NullPointerException("run the assign lift method before setting target position");

        double dynamicLowerLimit = -1 * Math.asin(config.getRearExtensionLimitInch() / (config.getRetractedLiftLengthInch() + lift.getTargetPosition()));
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
