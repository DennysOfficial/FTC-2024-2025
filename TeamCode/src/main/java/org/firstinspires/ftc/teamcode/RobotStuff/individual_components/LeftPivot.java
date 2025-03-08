package org.firstinspires.ftc.teamcode.RobotStuff.individual_components;

import android.util.Range;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
        return (config.inputMap == null) ? 0 : (float) config.inputMap.getPivotStick();
    }


    static final int encoderCountsPerRevMotor = 28;
    static final double finalGearRatio = (28.0 / 150.0) * (1. / 3.61) * (1. / 5.23); // rotations of final over rotations of encoder
    static final double encoderCountsPerRevFinal = encoderCountsPerRevMotor / finalGearRatio;
    static final double encoderCountsPerDeg = encoderCountsPerRevFinal / 360;

    static final double extendedLiftPosition = 30;
    public static double extendedGComp = 0.4;
    public static double retractedGComp = .1;

    public static double gCompAngleOffset = 0.1;


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
        motors.addMotor(config.deviceConfig.leftPivot, DcMotorSimple.Direction.FORWARD);
    }


    public LeftPivot(ControlMode defaultControlMode, OpMode opMode, RobotConfig config) {
        super(defaultControlMode, opMode, config, "LeftPivot", "Degrees", 1.0 / encoderCountsPerDeg);

        analogEncoder = opMode.hardwareMap.get(AnalogInput.class, config.deviceConfig.leftPivotAnalogEncoder);

        softLimits = new Range<>(-95.0, 50.0);
    }

    double previousLeftLiftPosition = Double.NaN;

    public void setTargetPosition(double targetPosition) {
        if (leftLift == null)
            throw new NullPointerException("run the assign lift method before setting target position");

        double dynamicLowerLimit = -1 * Math.asin(config.getRearExtensionLimitInch() / (leftLift.retractedRadius + leftLift.getPosition()));
        dynamicLowerLimit = Math.toDegrees(dynamicLowerLimit);
        targetPosition = MathUtils.clamp(targetPosition, dynamicLowerLimit, Double.POSITIVE_INFINITY);

        //opMode.telemetry.addData("pivotDynamicLimit", dynamicLowerLimit);
        super.setTargetPosition(targetPosition);
    }


    public static double velocityFeedforwardCoefficientRetracted = 0;
    public static double KpRetracted = 0.1;
    public static double KiRetracted = 0;
    public static double KdRetracted = 0.001;

    public static double velocityFeedforwardCoefficientExtended = 0;
    public static double KpExtended = 0.05;
    public static double KiExtended = 0;
    public static double KdExtended = 0.003;

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

    AnalogInput analogEncoder;

    double analogPosition;
    double analogError;
    double angleError;

    public static double initialAnalogPosition = 1.6;
    public static double analogRangeMax = 3.3;
    public static double analogRangeMin = 0;


    static double getAnalogRange() {
        return analogRangeMax - analogRangeMin;
    }

    static final double degreesOverAnalog = (360 / getAnalogRange()) * (15.0 / 150.0);
    static final double analogOverDegrees = 1.0 / degreesOverAnalog;

    public static double correctionFactor = 0;
    double angleCorrection = 0;
    double correctedAngle;

    double encoderCalculatedAnalogValue;

    @Override
    void miscUpdate() {

        double thing = (analogOverDegrees * (getPosition())) % getAnalogRange();

        if (getPosition() < 0)
            encoderCalculatedAnalogValue = getAnalogRange() + thing;
        else
            encoderCalculatedAnalogValue = initialAnalogPosition + ((thing > 0) ? thing : getAnalogRange() - thing);


        analogPosition = analogEncoder.getVoltage();
        analogPosition = MathUtils.clamp(analogPosition,0,getAnalogRange());


        analogError = analogPosition - encoderCalculatedAnalogValue;

        if (Math.abs(analogError) > getAnalogRange() * 0.5) {
            double goodAnalogError = getAnalogRange() - Math.abs(analogError);
            analogError = Math.copySign(goodAnalogError, -analogError);
        }

        angleCorrection += analogError * correctionFactor;

        //correctedAngle = getPosition() + angleCorrection;

        positionOffset = angleCorrection;

        if (config.debugConfig.getAnalogEncoderDebug()) {
            opMode.telemetry.addLine();
            opMode.telemetry.addLine();
            opMode.telemetry.addData(axisName + " predicted analog value", encoderCalculatedAnalogValue);
            opMode.telemetry.addData(axisName + " thing", thing);
            opMode.telemetry.addData(axisName + " analog value", analogPosition);
            opMode.telemetry.addData(axisName + " analog error", analogError);

            opMode.telemetry.addLine();
            opMode.telemetry.addData(axisName + " angle correction", angleCorrection);
            opMode.telemetry.addData(axisName + " correctedAngle", correctedAngle);
        }

    }

    double calculateTorqueGravity(double liftExtension) {
        double interpolationAmount = liftExtension / extendedLiftPosition;
        //opMode.telemetry.addData("interpolation amount", interpolationAmount);

        return Math.sin(gCompAngleOffset + Math.toRadians(getPosition())) * MathStuff.lerp(retractedGComp, extendedGComp, interpolationAmount);
    }
}
