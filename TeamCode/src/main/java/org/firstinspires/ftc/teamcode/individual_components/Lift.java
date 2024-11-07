package org.firstinspires.ftc.teamcode.individual_components;

import android.util.Range;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;

@Config
public class Lift extends ControlAxis {


    double pivotPosition;
    public static double gCompMultiplier = 0.1;

    public static double Kp = 0.8;
    public static double Ki = 0.02;
    public static double Kd = 0.03;

    public static double velocityFeedforwardCoefficient = 0;


    @Override
    protected void initMotors() {
        motors.addMotor(config.deviceConfig.rightLift, DcMotorSimple.Direction.FORWARD);
        // motors.addMotor(config.deviceConfig.leftLift, DcMotorSimple.Direction.REVERSE);

        //motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motors.getMotor(config.deviceConfig.leftLift).setMotorDisable();
    }

    @Override
    protected void updatePositionPIDCoefficients() {
        positionPID.setCoefficients(Kp, Ki, Kd);
    }


    public Lift(OpMode opMode, RobotConfig config) {
        super(opMode, config, "Lift", "inches", 27.0 / 4300.0);

        softLimits = new Range<>(0.5, 31.0);

        physicalLimits = new Range<>(0.0, Double.POSITIVE_INFINITY);
    }

    @Override
    public void setTargetPosition(double targetPosition) {

        double upperLimit = config.getFrontExtensionLimitInch() / Math.sin(Math.toRadians(pivotPosition)) - config.getRetractedLiftLengthInch();
        upperLimit = Math.abs(upperLimit);
        targetPosition = MathUtils.clamp(targetPosition, Double.NEGATIVE_INFINITY, upperLimit);

        opMode.telemetry.addData("liftDynamicLimit", upperLimit);

        super.setTargetPosition(targetPosition);
    }


    public void update(double deltaTime, double pivotAngleDeg) {
        updateEssentials();

        this.pivotPosition = pivotAngleDeg;

        switch (getControlMode()) {
            case positionControl:
                targetVelocity = config.inputMap.getLiftStick() * config.sensitivities.getLiftRate();

                updateVelocityControl(deltaTime, pivotAngleDeg);
                break;

            case positionControl:
                double positionFeedforward = -calcGravityForce(pivotAngleDeg);
                updatePositionPID(getTargetPosition(), positionFeedforward);
                break;

            case velocityControl:
                updateVelocityControl(deltaTime, pivotAngleDeg);
                break;

            case torqueControl:
                motors.setTorque(config.inputMap.getLiftStick() * config.sensitivities.getLiftSensitivity() - calcGravityForce(pivotAngleDeg), getVelocityTPS());
                break;

            case testing:

        }
    }


    void updateVelocityControl(double deltaTime, double pivotAngleDeg) {
        setTargetPosition(getTargetPosition() + targetVelocity * deltaTime);
        double directFeedforward = -calcGravityForce(pivotAngleDeg) + targetVelocity * velocityFeedforwardCoefficient;
        updatePositionPID(getTargetPosition(), directFeedforward);
    }

    public double calcGravityForce(double pivotAngleDeg) {
        return -Math.cos(Math.toRadians(pivotAngleDeg)) * gCompMultiplier;
    }
}
