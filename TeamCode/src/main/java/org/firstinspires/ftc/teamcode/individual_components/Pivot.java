package org.firstinspires.ftc.teamcode.individual_components;

import androidx.annotation.NonNull;
import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        motors.addMotor(config.deviceConfig.leftPivot, DcMotorSimple.Direction.REVERSE);
        motors.addMotor(config.deviceConfig.rightPivot, DcMotorSimple.Direction.REVERSE);

        motors.setTargetPosition(0);
        //motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    protected void updatePositionPIDCoefficients() {
        positionPID.setCoefficients(getKp(), getKi(), getKd());
    }


    public Pivot(OpMode opMode, RobotConfig config, ElapsedTime runtime) {
        super(opMode, config, "Pivot", "Degrees", 1.0 / encoderCountsPerDeg, runtime);

        upperLimit = 86.9;
        lowerLimit = -40;

    }

    @Override
    public void setTargetPosition(double targetPosition) {
        double dynamicLowerLimit = -1 * Math.asin(config.getRearExtensionLimitInch() / (config.getRetractedLiftLengthInch() + liftPosition));
        dynamicLowerLimit = Math.toDegrees(dynamicLowerLimit);
        targetPosition = MathUtils.clamp(targetPosition, dynamicLowerLimit, Double.POSITIVE_INFINITY);

        opMode.telemetry.addData("pivotDynamicLimit", dynamicLowerLimit);
        super.setTargetPosition(targetPosition);
    }

    @Override
    void runUpdate() {
        updateEssentials();

        liftPosition = lift.getPosition();

        if (config.debugConfig.pivotTorqueDebug()) {
            opMode.telemetry.addData("Pivot gravity", calculateTorqueGravity(liftPosition));
        }

        switch (getControlMode()) {

            case positionControl:
                double positionFeedforward = -calculateTorqueGravity(liftPosition);
                updatePositionPID(getTargetPosition(), positionFeedforward);
                break;

            case testing:
        }

        positionPID.setPreviousActualPosition(getPosition());
    }


    public class RunPID implements Action {
        Lift lift;

        public RunPID(Lift lift) {
            this.lift = lift;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        }
    }


    public void setNetTorque(double torque) {
        motors.setTorque(torque - calculateTorqueGravity(liftPosition), getVelocityTPS());
    }

    void updateVelocityControl(double deltaTime) {
        setTargetPosition(getTargetPosition() + targetVelocity * deltaTime);
        double velocityFeedforward = -calculateTorqueGravity(liftPosition) + targetVelocity * getVelocityFeedforwardCoefficient();
        updatePositionPID(getTargetPosition(), velocityFeedforward);
    }

    double calculateTorqueGravity(double liftExtension) {
        double interpolationAmount = liftExtension / extendedLiftPosition;
        opMode.telemetry.addData("interpolation amount", interpolationAmount);

        return Math.sin(Math.toRadians(getPosition())) * MathStuff.lerp(retractedGComp, extendedGComp, interpolationAmount);
    }

    public class GoToPosition implements Action {

        double targetPosition;

        public GoToPosition(double targetPosition) {
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setControlMode(ControlMode.positionControl);
            setTargetPosition(targetPosition);
            return false;
        }
    }

    public Action goToPosition(double targetPosition) {
        return new GoToPosition(targetPosition);
    }
}
