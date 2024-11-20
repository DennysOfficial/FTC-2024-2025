package org.firstinspires.ftc.teamcode.RobotStuff.individual_components;

import android.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.CustomPID;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.MultiTorqueMotor;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.PositionDerivatives;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ReadOnlyRuntime;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories.LinearTrajectory;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories.MotionState;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories.SinusoidalTrajectory;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories.Trajectory;

public abstract class ControlAxis {  //schrödinger's code

    ReadOnlyRuntime runtime;
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

    public ControlMode defaultControlMode;


    public enum ControlMode {
        disabled,
        torqueControl,
        positionControl,
        velocityControl,
        gamePadVelocityControl,
        gamePadTorqueControl,
        trajectoryControl,
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

            case trajectoryControl:
                if (activeTrajectory == null || !activeTrajectory.isActive())
                    break;
                setTargetPosition(getPosition());
                targetVelocity = 0;
                targetAcceleration = 0;
                this.controlMode = controlMode;
                motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

    /**
     * doesn't reset target position or anything like that
     */
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
    public double targetVelocity;

    public double getVelocityTPS() {
        return positionDerivatives.getVelocity() / unitsPerEncoderCount;
    }

    void updateVelocityControl() {
        setTargetPosition(getTargetPosition() + targetVelocity * deltaTime);
        updatePositionPID(getTargetPosition(), getStaticFeedforward(targetVelocity) + getVelocityFeedforward());
    }

    // Acceleration stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
    protected double targetAcceleration = 0;


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

    public ControlAxis(ControlMode defaultControlMode, OpMode opMode, RobotConfig config, String axisName, String unitName, double unitsPerEncoderCount, ReadOnlyRuntime runtime) {
        this.opMode = opMode;
        this.config = config;
        this.axisName = axisName;
        this.defaultControlMode = defaultControlMode;
        this.controlMode = defaultControlMode;

        this.unitName = unitName;
        this.unitsPerEncoderCount = unitsPerEncoderCount;
        this.runtime = runtime;

        motors = new MultiTorqueMotor(opMode.hardwareMap, config.sensorData);

        initPid();
        initMotors();

        positionDerivatives = new PositionDerivatives(getPosition());
    }

    // trajectory stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

    Trajectory activeTrajectory;

    public void linearMoveToPosition(double targetPosition, double duration) {
        activeTrajectory = new LinearTrajectory(runtime, getPosition(), targetPosition, duration);
        setControlMode(ControlMode.trajectoryControl);
    }

    public void fancyMoveToPosition(double targetPosition, double duration) {
        activeTrajectory = new SinusoidalTrajectory(runtime, getPosition(), targetPosition, duration);
        setControlMode(ControlMode.trajectoryControl);
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

        if (config.inputMap.getUnAbort())
            controlMode = defaultControlMode;


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

            case trajectoryControl:
                if (!activeTrajectory.isActive()) {
                    setControlModeUnsafe(defaultControlMode);
                    break;
                }
                MotionState targetMotionState = activeTrajectory.sampleTrajectory();

                MotionState.telemetryMotionState(opMode.telemetry, targetMotionState, axisName + " target");

//                setTargetPosition(targetMotionState.position);
//                targetVelocity = targetMotionState.velocity;
//                targetAcceleration = targetMotionState.acceleration;

                updatePositionPID(getTargetPosition(), getStaticFeedforward(targetVelocity) + getVelocityFeedforward() + getAccelerationFeedforward());
                break;

            case testing:
        }
        miscUpdate();
        debugUpdate();
    }
}
