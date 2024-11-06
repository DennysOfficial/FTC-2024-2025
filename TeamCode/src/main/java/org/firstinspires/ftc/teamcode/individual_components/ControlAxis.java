package org.firstinspires.ftc.teamcode.individual_components;

import androidx.annotation.NonNull;
import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.motionControl.CustomPID;
import org.firstinspires.ftc.teamcode.motionControl.MultiTorqueMotor;
import org.firstinspires.ftc.teamcode.motionControl.PositionDerivatives;

public abstract class ControlAxis {

    ElapsedTime runtime;
    double deltaTime = 0;
    public static double staticThresholdUnitsPerSec = 0;
    public static double staticFrictionComp = 0;
    public static double kineticFrictionComp = 0;


    public final String axisName;
    public final String unitName;

    public final double unitsPerEncoderCount;


    protected OpMode opMode;
    protected RobotConfig config;


    protected MultiTorqueMotor motors;

    protected abstract void initMotors();

    public enum ControlMode {
        disabled,
        torqueControl,
        positionControl,
        velocityControl,
        gamepadVelocityControl,
        testing
    }

    private ControlMode controlMode = ControlMode.disabled;

    public void setControlMode(ControlMode controlMode) {
        if (this.controlMode == controlMode)
            return;

        switch (controlMode) {
            case disabled:
                motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motors.setPower(0);
                break;

            case velocityControl:
                setTargetPosition(getPosition());
                this.controlMode = ControlMode.velocityControl;
                motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;

            case positionControl:
                this.controlMode = ControlMode.positionControl;
                motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;
            case gamepadVelocityControl:
                setTargetPosition(getPosition());
                this.controlMode = ControlMode.gamepadVelocityControl;
                motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            default:
                this.controlMode = controlMode;
        }
    }

    public void setControlModeUnsafe(ControlMode controlMode) {
        this.controlMode = controlMode;
    }

    public ControlMode getControlMode() {
        return controlMode;
    }


    protected PositionDerivatives positionDerivatives;


    protected CustomPID positionPID;

    void initPid() {
        positionPID = new CustomPID(opMode, config, axisName + " positionPID");
        updatePIDCoefficients();
    }

    abstract double getKp();

    abstract double getKi();

    abstract double getKd();

    abstract double getStaticFeedforward(double targetDirection);

    double staticFrictionForce(double targetDirection, double velocity) {
        if (Math.abs(velocity) > staticThresholdUnitsPerSec)
            return 0;

        return staticFrictionComp * Math.copySign(1, -targetDirection);
    }


    abstract double getVelocityFeedforward();

    double kineticFrictionForce(double targetDirection, double velocity) {
        if (Math.abs(velocity) <= staticThresholdUnitsPerSec)
            return 0;
        return kineticFrictionComp * Math.copySign(1, velocity);
    }


    abstract double getAccelerationFeedforward();

    void updatePIDCoefficients() {
        positionPID.setCoefficients(getKp(), getKi(), getKd());
    }


    protected double targetPosition = 0;

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = MathUtils.clamp(targetPosition, lowerLimit, upperLimit);
    }

    protected double upperLimit = Double.POSITIVE_INFINITY;
    protected double lowerLimit = Double.NEGATIVE_INFINITY;
    protected double physicalUpperLimit = Double.POSITIVE_INFINITY;
    protected double physicalLowerLimit = Double.NEGATIVE_INFINITY;
    protected double positionOffset = 0;


    /**
     * adjusts the position offset so that the current position doesn't extend past what the physical mechanism is known to be capable of. Mainly compensating for belt skipping on the lifts
     */
    void adjustOffsetForPhysicalLimits() {
        if (getPosition() > physicalUpperLimit) {
            positionOffset -= getPosition() - physicalUpperLimit;
        } else if (getPosition() < physicalLowerLimit) {
            positionOffset -= getPosition() - physicalLowerLimit;
        }
    }

    double targetVelocity;

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    double targetTorque;

    public double getTargetTorque() {
        return targetTorque;
    }

    void setTargetTorque(double targetTorque) {
        this.targetTorque = targetTorque;
    }

    public ControlAxis(OpMode opMode, RobotConfig config, String axisName, String unitName, double unitsPerEncoderCount, ElapsedTime runtime) {
        this.opMode = opMode;
        this.config = config;
        this.axisName = axisName;

        this.unitName = unitName;
        this.unitsPerEncoderCount = unitsPerEncoderCount;
        this.runtime = runtime;

        motors = new MultiTorqueMotor(opMode.hardwareMap, config.sensorData);

        initPid();
        initMotors();

        positionDerivatives = new PositionDerivatives(getPosition());

        updatePositionPIDCoefficients();
    }

    double previousTime = Double.NaN;

    void updateDeltaTime() {
        if (Double.isNaN(previousTime)) {
            previousTime = runtime.seconds();
            deltaTime = 0;
            return;
        }

        deltaTime = -1 * previousTime + (previousTime = runtime.seconds());
    }


    protected void updateEssentials() {
        //opMode.telemetry.addData()
        updateDeltaTime();
        positionDerivatives.update(getPosition(), deltaTime);

        //adjustOffsetForPhysicalLimits();

        if (config.inputMap.getAbort())
            controlMode = ControlMode.disabled;

        if (controlMode == ControlMode.disabled) {
            motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors.setPower(0);
        }

        if (config.debugConfig.getControlModeDebug())
            opMode.telemetry.addData(axisName + "ControlMode", controlMode.toString());

        if (config.debugConfig.getAllPositionDebug())
            opMode.telemetry.addData(axisName + " position " + unitName, getPosition());

    }

    abstract void runUpdate();

    abstract public Action update();

    protected void updatePositionPID(double targetPosition, double feedforward) {
        updatePIDCoefficients();

        double targetTorque = positionPID.runPID(targetPosition, getPosition(), deltaTime);
        targetTorque += feedforward;

        motors.setTorque(targetTorque, positionDerivatives.getVelocity() / unitsPerEncoderCount);
    }

    void updateVelocityControl() {
        setTargetPosition(getTargetPosition() + targetVelocity * deltaTime);
        updatePositionPID(getTargetPosition(), getStaticFeedforward(targetVelocity) + getVelocityFeedforward());
    }
    /**
     * returns the current position of the axis in the designated units
     */
    public double getPosition() {
        return motors.getCurrentPosition() * unitsPerEncoderCount + positionOffset;
    }


    public double getVelocityTPS() {
        return positionDerivatives.getVelocity() / unitsPerEncoderCount;
    }


    public class SetPosition implements Action {
        double targetPosition;

        public SetPosition(double targetPosition) {
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setTargetPosition(targetPosition);
            setControlMode(ControlMode.positionControl);
            return false;
        }
    }

    public Action setPosition(double targetPosition) {
        return new SetPosition(targetPosition);
    }


    public class SetTorque implements Action {
        double targetTorque;

        public SetTorque(double targetTorque) {
            this.targetTorque = targetTorque;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setTargetTorque(targetTorque);
            setControlMode(ControlMode.torqueControl);
            return false;
        }
    }

    public Action setTorque(double targetTorque) {
        return new SetTorque(targetTorque);
    }


}
