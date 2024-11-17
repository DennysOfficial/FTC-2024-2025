package org.firstinspires.ftc.teamcode.RobotStuff.individual_components;

import android.util.Range;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.CustomPID;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.MultiTorqueMotor;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.PositionDerivatives;

public abstract class ControlAxis {  //schrödinger's code

    ElapsedTime runtime;
    double deltaTime = 0;

    protected PositionDerivatives positionDerivatives;

    public final String axisName;
    public final String unitName;

    public final double unitsPerEncoderCount;


    protected OpMode opMode;
    protected RobotConfig config;

    // input stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
    abstract float getInput();

    abstract float getVelocityControlMaxRate(); //units per second

    abstract float getTorqueControlSensitivity();

    // motor stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

    protected MultiTorqueMotor motors;

    protected abstract void initMotors();


    // control mode stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
    private ControlMode controlMode = ControlMode.disabled;

    public enum ControlMode {
        disabled,
        torqueControl,
        positionControl,
        velocityControl,
        gamePadVelocityControl,
        gamePadTorqueControl,
        testing
    }

    public void setControlMode(ControlMode controlMode) {
        if (this.controlMode == controlMode)
            return;

        switch (controlMode) {
            case disabled:
                motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motors.setPower(0);
                break;

            case gamePadVelocityControl:
            case velocityControl:
                setTargetPosition(getPosition());
                this.controlMode = controlMode;
                motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;


            case positionControl:
                this.controlMode = ControlMode.positionControl;
                motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;

            case gamePadTorqueControl:
            case torqueControl:
                this.controlMode = controlMode;
                motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;

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

    // PID stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
    protected CustomPID positionPID;

    void initPid() {
        positionPID = new CustomPID(opMode.telemetry, config, axisName + " positionPID");
        // updatePIDCoefficients();
    }

    abstract double getKp();

    abstract double getKi();

    abstract double getKd();

    void updatePIDCoefficients() {
        positionPID.setCoefficients(getKp(), getKi(), getKd());
    }

    /**
     * ALREADY HAS STATIC FEEDFORWARD
     *
     * @param feedforward ALREADY HAS STATIC FEEDFORWARD
     */
    protected void updatePositionPID(double targetPosition, double feedforward) {
        updatePIDCoefficients();

        double targetTorque = positionPID.runPID(targetPosition, getPosition(), deltaTime);
        targetTorque += feedforward;
        targetTorque += getStaticFeedforward(targetTorque);

        motors.setTorque(targetTorque, positionDerivatives.getVelocity() / unitsPerEncoderCount);
    }

    // feedforward stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
    abstract double getStaticFeedforward(double targetDirection);

    public double staticFrictionForce(double targetDirection, double staticFrictionComp, double staticThresholdUnitsPerSec) {
        if (Math.abs(positionDerivatives.getVelocity()) > staticThresholdUnitsPerSec)
            return 0;

        return staticFrictionComp * Math.copySign(1, -targetDirection);
    }

    abstract double getVelocityFeedforward();

    public double kineticFrictionForce(double targetDirection, double kineticFrictionComp, double staticThresholdUnitsPerSec) {
        if (Math.abs(positionDerivatives.getVelocity()) <= staticThresholdUnitsPerSec)
            return 0;
        return kineticFrictionComp * Math.copySign(1, positionDerivatives.getVelocity());
    }

    abstract double getAccelerationFeedforward();

    // Position stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
    protected double targetPosition = 0;

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = softLimits.clamp(targetPosition);
    }

    Range<Double> softLimits = new Range<Double>(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    Range<Double> physicalLimits = new Range<Double>(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

    protected double positionOffset = 0;

    /**
     * returns the current position of the axis in the designated units
     */
    public double getPosition() {
        return motors.getCurrentPosition() * unitsPerEncoderCount + positionOffset;
    }

    /**
     * adjusts the position offset so that the current position doesn't extend past what the physical mechanism is known to be capable of. Mainly compensating for belt skipping on the lifts maybe
     */
    void adjustOffsetForPhysicalLimits() {
        if (getPosition() > physicalLimits.getUpper()) {
            positionOffset -= getPosition() - physicalLimits.getUpper();
        } else if (getPosition() < physicalLimits.getLower()) {
            positionOffset -= getPosition() - physicalLimits.getLower();
        }
    }


    // Velocity stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
    double targetVelocity;

    public double getVelocityTPS() {
        return positionDerivatives.getVelocity() / unitsPerEncoderCount;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    void updateVelocityControl() {
        setTargetPosition(getTargetPosition() + targetVelocity * deltaTime);
        updatePositionPID(getTargetPosition(), getStaticFeedforward(targetVelocity) + getVelocityFeedforward());
    }


    // Torque stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
    double targetTorque;

    public double getTargetTorque() {
        return targetTorque;
    }

    public void setTargetTorque(double targetTorque) {
        this.targetTorque = targetTorque;
    }

    // deltaTime stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
    double previousTime = 0;

    void updateDeltaTime() {
        if (previousTime == 0) {
            previousTime = runtime.seconds();
            deltaTime = 0;
            return;
        }

        deltaTime = -1 * previousTime + (previousTime = runtime.seconds());
    }

    // Constructor stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

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
    }

    // update stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

    abstract void miscUpdate();

    protected void debugUpdate() {
        //opMode.telemetry.addData()

        //adjustOffsetForPhysicalLimits();

        if (config.debugConfig.getControlModeDebug())
            opMode.telemetry.addData(axisName + "ControlMode", controlMode.toString());

        if (config.debugConfig.getAllPositionDebug())
            opMode.telemetry.addData(axisName + " position " + unitName, getPosition());
    }

    public void update() {
        updateDeltaTime();
        positionDerivatives.update(getPosition(), deltaTime);

        if (config.inputMap.getAbort())
            controlMode = ControlMode.disabled;

        if (controlMode == ControlMode.disabled) {
            motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors.setPower(0);
        }

        switch (getControlMode()) {
            case gamePadVelocityControl:
                targetVelocity = getInput() * getVelocityControlMaxRate();
                updateVelocityControl();
                break;

            case positionControl:
                updatePositionPID(getTargetPosition(), 0);
                break;

            case velocityControl:
                updateVelocityControl();
                break;

            case torqueControl:
                motors.setTorque(targetTorque + getStaticFeedforward(targetTorque), getVelocityTPS());
                break;
            case gamePadTorqueControl:
                targetTorque = getInput() * getTorqueControlSensitivity();
                motors.setTorque(targetTorque + getStaticFeedforward(targetTorque), getVelocityTPS());
                break;

            case testing:
        }
        miscUpdate();
        debugUpdate();
    }
}
