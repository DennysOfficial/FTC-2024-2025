package org.firstinspires.ftc.teamcode.RobotStuff.individual_components;

import android.util.Range;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.MathStuff;

@Config
public class LeftPivot extends ControlAxis {



    LeftLift leftLift;

    public void assignLift(LeftLift leftLift) {
        if (leftLift == null)
            throw new NullPointerException("the lift you tried to assign is null you goober");
        this.leftLift = leftLift;
    }

    @Override
    float getInput() {
        return (config.inputMap == null) ? 0 : (float) config.inputMap.getRightPivotStick();
    }


    static final int encoderCountsPerRevMotor = 28;
    static final double finalGearRatio = (28.0/150.0)*(1./4.)*(1./3.); // rotations of final over rotations of encoder
    static final double encoderCountsPerRevFinal = encoderCountsPerRevMotor / finalGearRatio;
    static final double encoderCountsPerDeg = encoderCountsPerRevFinal / 360;

    static final double extendedLiftPosition = 30;
    public static double extendedGComp = 0.2;
    public static double retractedGComp = 0.05;

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
        motors.addMotor(config.deviceConfig.leftPivot, DcMotorSimple.Direction.REVERSE);
    }


    public LeftPivot(ControlMode defaultControlMode, OpMode opMode, RobotConfig config) {
        super(defaultControlMode, opMode, config, "LeftPivot", "Degrees", 1.0 / encoderCountsPerDeg);

        softLimits = new Range<>(-95.0, 50.0);
    }

    double previousRightLiftTargetPosition = Double.NaN;

    public void setTargetPosition(double targetPosition) {
        if (leftLift == null)
            throw new NullPointerException("run the assign lift method before setting target position");

        if (targetPosition == getTargetPosition() && previousRightLiftTargetPosition == (previousRightLiftTargetPosition = leftLift.getTargetPosition()))
            return;

        double dynamicLowerLimit = -1 * Math.asin(config.getRearExtensionLimitInch() / (leftLift.retractedRadius + leftLift.getTargetPosition()));
        dynamicLowerLimit = Math.toDegrees(dynamicLowerLimit);
        targetPosition = MathUtils.clamp(targetPosition, dynamicLowerLimit, Double.POSITIVE_INFINITY);

        //opMode.telemetry.addData("pivotDynamicLimit", dynamicLowerLimit);
        super.setTargetPosition(targetPosition);

    }


    public static double velocityFeedforwardCoefficientRetracted = 0;
    public static double KpRetracted = 0.05;
    public static double KiRetracted = 0;
    public static double KdRetracted = 0.0015;

    public static double velocityFeedforwardCoefficientExtended = 0;
    public static double KpExtended = 0.05;
    public static double KiExtended = 0;
    public static double KdExtended = 0.0045;

    @Override
    double getKp() {
        if (leftLift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return MathStuff.lerp(KpRetracted, KpExtended, leftLift.getPosition() / extendedLiftPosition);
    }

    @Override
    double getKi() {
        if (leftLift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return MathStuff.lerp(KiRetracted, KiExtended, leftLift.getPosition() / extendedLiftPosition);
    }


    @Override
    double getKd() {
        if (leftLift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return MathStuff.lerp(KdRetracted, KdExtended, leftLift.getPosition() / extendedLiftPosition);
    }

    @Override
    double getStaticFeedforward(double targetDirection) {
        if (leftLift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return -calculateTorqueGravity(leftLift.getPosition());
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
        if (leftLift == null)
            throw new NullPointerException("run the assign lift method before running anything else");
        return MathStuff.lerp(velocityFeedforwardCoefficientRetracted, velocityFeedforwardCoefficientExtended, leftLift.getPosition() / extendedLiftPosition);
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
