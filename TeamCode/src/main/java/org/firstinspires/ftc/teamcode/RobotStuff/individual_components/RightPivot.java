package org.firstinspires.ftc.teamcode.RobotStuff.individual_components;

import android.util.Range;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
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

        analogEncoder = opMode.hardwareMap.get(AnalogInput.class, config.deviceConfig.rightPivotAnalogEncoder);

        softLimits = new Range<>(-60.0, 97.0);
    }

    double previousRightLiftTargetPosition = Double.NaN;

    public void setTargetPosition(double targetPosition) {
        if (rightLift == null)
            throw new NullPointerException("run the assign lift method before setting target position");

        double dynamicLowerLimit = -1 * Math.asin(config.getRearExtensionLimitInch() / (rightLift.retractedExtension + rightLift.getPosition()));
        dynamicLowerLimit = Math.toDegrees(dynamicLowerLimit);
        targetPosition = MathUtils.clamp(targetPosition, dynamicLowerLimit, Double.POSITIVE_INFINITY);

        //opMode.telemetry.addData("pivotDynamicLimit", dynamicLowerLimit);
        super.setTargetPosition(targetPosition);

    }


    public static double velocityFeedforwardCoefficientRetracted = 0;
    public static double KpRetracted = 0.2;
    public static double KiRetracted = 0.0;
    public static double KdRetracted = 0.003;

    public static double velocityFeedforwardCoefficientExtended = 0;
    public static double KpExtended = 0.2;
    public static double KiExtended = 0.0;
    public static double KdExtended = 0.005;

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


    AnalogInput analogEncoder;

    double analogPosition;
    double analogError;
    double angleError;

    public static double initialAnalogPosition = 0.169;
    public static double analogRangeMax = 3.3;
    public static double analogRangeMin = 0;


    static double getAnalogRange() {
        return analogRangeMax - analogRangeMin;
    }

    static final double degreesOverAnalog = (360 / getAnalogRange()) * (15.0 / 150.0);
    static final double analogOverDegrees = 1.0 / degreesOverAnalog;

    public static double correctionFactor = 0.05;
    double angleCorrection = 0;
    double correctedAngle;

    double encoderCalculatedAnalogValue;

    @Override
    void miscUpdate() {
        encoderCalculatedAnalogValue = Math.abs((initialAnalogPosition + analogOverDegrees * (getPosition()) % getAnalogRange()));

        if (getPosition() < 0)
            encoderCalculatedAnalogValue = getAnalogRange() - encoderCalculatedAnalogValue;

        analogPosition = analogEncoder.getVoltage();

        analogError = analogPosition - encoderCalculatedAnalogValue;

        if (Math.abs(analogError) > getAnalogRange() * 0.5) {
            double goodAnalogError = getAnalogRange() - Math.abs(analogError);
            analogError = Math.copySign(goodAnalogError, -analogError);
        }

        if (getPosition() > 20)
            angleCorrection += analogError * correctionFactor;

            correctedAngle = getPosition() + angleCorrection;

        positionOffset = angleCorrection;

        if (config.debugConfig.getAnalogEncoderDebug()) {
            opMode.telemetry.addLine();
            opMode.telemetry.addLine();
            opMode.telemetry.addData(axisName + "analog value", analogPosition);
            opMode.telemetry.addData(axisName + "analog error", analogError);

            opMode.telemetry.addLine();
            opMode.telemetry.addData(axisName + "angle correction", angleCorrection);
            opMode.telemetry.addData(axisName + "correctedAngle", correctedAngle);
        }

//        opMode.telemetry.addLine();
//        opMode.telemetry.addLine();
//        opMode.telemetry.addData("gravity comp torque", calculateTorqueGravity());
//        opMode.telemetry.addData("acceleration comp torque", calculateTorqueAcceleration());
    }

    double calculateTorqueGravity(double liftExtension) {
        double interpolationAmount = liftExtension / extendedLiftPosition;
        //opMode.telemetry.addData("interpolation amount", interpolationAmount);

        return Math.sin(gCompAngleOffset + Math.toRadians(getPosition())) * MathStuff.lerp(retractedGComp, extendedGComp, interpolationAmount);
    }

    double calculateTorqueGravity() {
        double interpolationAmount = rightLift.getPosition() / extendedLiftPosition;
        //opMode.telemetry.addData("interpolation amount", interpolationAmount);

        return Math.sin(gCompAngleOffset + Math.toRadians(getPosition())) * MathStuff.lerp(retractedGComp, extendedGComp, interpolationAmount);
    }

    double calculateTorqueAcceleration() {
        double accelerationCompCoefficient = MathStuff.lerp(retractedGComp, extendedGComp, rightLift.getPosition() / extendedLiftPosition);

        double accelerationTorque = -config.sensorData.getUpAcceleration() * Math.sin(gCompAngleOffset + Math.toRadians(getPosition())) * accelerationCompCoefficient;
        accelerationTorque -= config.sensorData.getForwardAcceleration() * Math.cos(gCompAngleOffset + Math.toRadians(getPosition())) * accelerationCompCoefficient;

        return accelerationTorque;
    }
}
