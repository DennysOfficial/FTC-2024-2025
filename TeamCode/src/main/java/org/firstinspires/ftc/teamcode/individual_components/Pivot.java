package org.firstinspires.ftc.teamcode.individual_components;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.MathStuff;

@Config
public class Pivot extends ControlAxis {


    public double liftPosition;

    static final int encoderCountsPerRevMotor = 28;
    static final double finalGearRatio = 1. / 200.; // rotations of final over rotations of motor
    static final double encoderCountsPerRevFinal = encoderCountsPerRevMotor / finalGearRatio;
    static final double encoderCountsPerDeg = encoderCountsPerRevFinal / 360;

    static final double extendedLiftPosition = 30;
    public static double extendedGComp = 0.2;
    public static double retractedGComp = 0.12;

    double getKp() {
        return MathStuff.lerp(KpRetracted, KpExtended, liftPosition / extendedLiftPosition);
    }

    double getKi() {
        return MathStuff.lerp(KiRetracted, KiExtended, liftPosition / extendedLiftPosition);
    }

    double getKd() {
        return MathStuff.lerp(KdRetracted, KdExtended, liftPosition / extendedLiftPosition);
    }

    double getVelocityFeedforwardCoefficient() {
        return MathStuff.lerp(velocityFeedforwardCoefficientRetracted, velocityFeedforwardCoefficientExtended, liftPosition / extendedLiftPosition);
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
    protected void initMotors() {
        motors.addMotor(config.deviceConfig.leftPivot, DcMotorSimple.Direction.FORWARD);
        motors.addMotor(config.deviceConfig.rightPivot, DcMotorSimple.Direction.FORWARD);

        motors.setTargetPosition(0);
        //motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    protected void updatePositionPIDCoefficients() {
        positionPID.setCoefficients(getKp(), getKi(), getKd());
    }


    public Pivot(OpMode opMode, RobotConfig config) {
        super(opMode, config, "Pivot", "Degrees", 1.0 / encoderCountsPerDeg);

        upperLimit = 86.9;
        lowerLimit = -40;

    }

    @Override
    public void setTargetPosition(double targetPosition) {
        double dynamicLowerLimit = -1* Math.asin(config.getRearExtensionLimitInch() / (config.getRetractedLiftLengthInch() + liftPosition));
        dynamicLowerLimit = Math.toDegrees(dynamicLowerLimit);
        targetPosition = MathUtils.clamp(targetPosition, dynamicLowerLimit, Double.POSITIVE_INFINITY);

        opMode.telemetry.addData("pivotDynamicLimit", dynamicLowerLimit);
        super.setTargetPosition(targetPosition);
    }


    public void update(double deltaTime, double liftPosition) {
        updateEssentials(deltaTime);

        this.liftPosition = liftPosition;



        if (config.debugConfig.pivotTorqueDebug()) {
            opMode.telemetry.addData("Pivot gravity", calculateTorqueGravity(liftPosition));
        }

        switch (controlMode) {
            case directControl:
                targetVelocity = config.inputMap.getPivotStick() * config.sensitivities.getPivotRate();
                updateVelocityControl(deltaTime);
                break;

            case positionControl:
                double positionFeedforward = -calculateTorqueGravity(liftPosition);
                updatePositionPID(getTargetPosition(), deltaTime, positionFeedforward);
                break;

            case velocityControl:
                updateVelocityControl(deltaTime);
                break;

            case directTorqueControl:
                motors.setTorque(config.inputMap.getPivotStick() * config.sensitivities.getPivotSensitivity() - calculateTorqueGravity(liftPosition), getVelocityTPS());
                break;

            case testing:

        }

        positionPID.setPreviousActualPosition(getPosition());
    }

    public void setNetTorque(double torque){
        motors.setTorque(torque - calculateTorqueGravity(liftPosition), getVelocityTPS());
    }

    void updateVelocityControl(double deltaTime) {
        setTargetPosition(getTargetPosition() + targetVelocity * deltaTime);
        double velocityFeedforward = -calculateTorqueGravity(liftPosition) + targetVelocity * getVelocityFeedforwardCoefficient();
        updatePositionPID(getTargetPosition(), deltaTime, velocityFeedforward);
    }

    double calculateTorqueGravity(double liftExtension) {
        double interpolationAmount = liftExtension / extendedLiftPosition;
        opMode.telemetry.addData("interpolation amount", interpolationAmount);

        return Math.sin(Math.toRadians(getPosition())) * MathStuff.lerp(retractedGComp, extendedGComp, interpolationAmount);
    }
}
