package org.firstinspires.ftc.teamcode.RobotStuff.individual_components;

import android.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.CustomPID;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.StopWatch;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.fancyMotorThings.MultiMotor;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.fancyMotorThings.MultiTorqueMotor;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.PositionDerivatives;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories.LinearTrajectory;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories.MotionState;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories.SinusoidalTrajectory;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories.Trajectory;

public abstract class ControlAxis {  //schrödinger's code

    public double deltaTime = 0;

    protected PositionDerivatives positionDerivatives;

    public final String axisName;
    public final String unitName;

    public double unitsPerEncoderCount;


    protected OpMode opMode;
    protected RobotConfig config;

    // input stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
    abstract float getInput();

    abstract float getVelocityControlMaxRate(); //units per second

    abstract float getTorqueControlSensitivity();

    // motor stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

    protected MultiMotor motors;

    protected abstract void addMotors();

    int getEncoder() {
        return motors.getCurrentPosition();
    }

    void setPower(double power) {
        if (config.debugConfig.getMotorPowerDebug())
            opMode.telemetry.addData(positionPID.instanceName + " motor power:", power);
        motors.setPower(power);
    }


    // control mode stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
    private ControlMode controlMode = ControlMode.disabled;

    public ControlMode defaultControlMode;

    ControlAxis leaderControlAxis;

    double leaderPositionOffset = 0;

    public void assignLeaderControlAxis(ControlAxis leaderControlAxis) {
        this.leaderControlAxis = leaderControlAxis;
    }

    public void assignLeaderControlAxis(ControlAxis leaderControlAxis, double positionOffset) {
        this.leaderControlAxis = leaderControlAxis;
        this.positionOffset = positionOffset;
    }


    public enum ControlMode {
        disabled,
        torqueControl,
        positionControl,
        velocityControl,
        gamePadVelocityControl,
        gamePadTorqueControl,
        trajectoryControl,
        followTheLeader,
        testing
    }

    enum HomingState {
        initHoming,
        retracting,
        dwelling,
        finishingHoming,
        homed,
    }

    HomingState homingState = HomingState.homed;

    public void setControlMode(ControlMode controlMode) {
        if (this.controlMode == controlMode)
            return;

        switch (controlMode) {
            case disabled:
                motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                setPower(0);
                break;

            case trajectoryControl:
                if (activeTrajectory == null || !activeTrajectory.isActive())
                    break;
                setTargetPosition(getNonCachedPosition());
                targetVelocity = 0;
                targetAcceleration = 0;
                this.controlMode = controlMode;
                motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;

            case gamePadVelocityControl:
            case velocityControl:
                targetPosition = getPosition();
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

            case followTheLeader:

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

    void updateCustomPIDCoefficients() {
        positionPID.setCoefficients(getKp(), getKi(), getKd());
    }

    void updateBuiltInPIDCoefficients() {
        positionPID.setCoefficients(getKp() * unitsPerEncoderCount, getKi() * unitsPerEncoderCount, getKd() * unitsPerEncoderCount);
    }

    /**
     * ALREADY HAS STATIC FEEDFORWARD
     *
     * @param nonStaticFeedforward ALREADY HAS STATIC FEEDFORWARD
     */
    protected void updatePositionPID(double targetPosition, double nonStaticFeedforward) {
        updateStopwatch.addTimeToTelemetryAndReset(opMode.telemetry, "stuff before updating position PID time");
        updateCustomPIDCoefficients();

        double targetTorque = positionPID.runPID(targetPosition, getPosition(), deltaTime);
        updateStopwatch.addTimeToTelemetryAndReset(opMode.telemetry, "Position PID time");
        targetTorque += getStaticFeedforward(targetTorque);
        targetTorque += nonStaticFeedforward;


        setPower(targetTorque);
        updateStopwatch.addTimeToTelemetryAndReset(opMode.telemetry, "set torque time");

        //motors.setTargetPosition((int) (targetPosition / unitsPerEncoderCount));
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
        if (Double.isNaN(targetPosition)) {
            return;
            //throw new ArithmeticException("target Position cant be nan you goober");
        }
        this.targetPosition = softLimits.clamp(targetPosition);
    }

    Range<Double> softLimits = new Range<Double>(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    Range<Double> physicalLimits = new Range<Double>(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

    protected double positionOffset = 0;

    /**
     * returns the current position of the axis in the designated units
     */
    public double getNonCachedPosition() {
        return getEncoder() * unitsPerEncoderCount;
    }

    double cachedPosition;

    public void updateCachedPosition() {
        cachedPosition = getNonCachedPosition();
    }

    /**
     * returns the current position of the axis in the designated units
     */
    public double getPosition() {
        return cachedPosition + positionOffset;
    }

    /**
     * adjusts the position offset so that the current position doesn't extend past what the physical mechanism is known to be capable of. Mainly compensating for belt skipping on the lifts, maybe, idk bro
     */
    void adjustOffsetForPhysicalLimits() {
        if (getPosition() > physicalLimits.getUpper()) {
            positionOffset -= getPosition() - physicalLimits.getUpper();
        } else if (getPosition() < physicalLimits.getLower()) {
            positionOffset -= getPosition() - physicalLimits.getLower();
        }
    }

    public void setCurrentPosition(double position) {
        positionOffset = position - getNonCachedPosition();
    }


    // Velocity stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
    public double targetVelocity;

    public double getVelocityTPS() {
        return positionDerivatives.getVelocity() / unitsPerEncoderCount;
    }

    void updateVelocityControl() {
        setTargetPosition(targetPosition + targetVelocity * deltaTime);
        updatePositionPID(targetPosition, getStaticFeedforward(targetVelocity) + getVelocityFeedforward());
    }

    // Acceleration stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
    protected double targetAcceleration = 0;


    // Torque stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
    public double targetTorque;

    // deltaTime stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
    double previousTime = 0;

    void updateDeltaTime() {
        if (previousTime == 0) {
            previousTime = System.nanoTime() / ((double) ElapsedTime.SECOND_IN_NANO);
            deltaTime = 0;
            return;
        }

        deltaTime = -1 * previousTime + (previousTime = System.nanoTime() / ((double) ElapsedTime.SECOND_IN_NANO));
    }

    // Constructor stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

    public ControlAxis(ControlMode defaultControlMode, OpMode opMode, RobotConfig config, String axisName, String unitName, double unitsPerEncoderCount) {
        this.opMode = opMode;
        this.config = config;
        this.axisName = axisName;
        this.unitName = unitName;
        this.unitsPerEncoderCount = unitsPerEncoderCount;
        this.defaultControlMode = defaultControlMode;

        motors = new MultiTorqueMotor(opMode.hardwareMap, config.sensorData);
        addMotors();

        updateCachedPosition();

        setControlMode(defaultControlMode);

        initPid();
        positionDerivatives = new PositionDerivatives(getPosition());
    }

    // trajectory stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

    Trajectory activeTrajectory;

    public void linearMoveToPosition(double targetPosition, double duration) {
        opMode.telemetry.addData("sending " + axisName + " to ", targetPosition);
        activeTrajectory = new LinearTrajectory(getPosition(), targetPosition, duration);
        setControlMode(ControlMode.trajectoryControl);
    }

    public void fancyMoveToPosition(double targetPosition, double duration) {
        opMode.telemetry.addData("sending " + axisName + " to ", targetPosition);
        activeTrajectory = new SinusoidalTrajectory(getPosition(), targetPosition, duration);
        setControlMode(ControlMode.trajectoryControl);
    }

    public boolean isBusy() {
        return activeTrajectory != null && activeTrajectory.isActive();
    }

    // update stuff \/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/

    abstract void miscUpdate();

    protected void debugUpdate() {
        if (config.debugConfig.getStateDebug())
            opMode.telemetry.addData(axisName + "ControlMode", controlMode.toString());

        if (config.debugConfig.positionDerivativesDebug())
            opMode.telemetry.addData(axisName + " motorPower", motors.getPower());


        if (config.debugConfig.getAllPositionDebug())
            opMode.telemetry.addData(axisName + " position " + unitName, getPosition());

        if (config.debugConfig.positionDerivativesDebug()) {
            opMode.telemetry.addData(axisName + " velocity " + unitName + "/sec", positionDerivatives.getVelocity());
            opMode.telemetry.addData(axisName + " acceleration " + unitName + "/sec^2", positionDerivatives.getAcceleration());
        }
    }

    StopWatch updateStopwatch = new StopWatch();

    public void update() {
        updateStopwatch.reset();
        updateStopwatch.debug = config.debugConfig.getTimeBreakdownDebug();

        updateDeltaTime();
        updateCachedPosition();
        positionDerivatives.update(getPosition(), deltaTime);


        if (config.inputMap != null && config.inputMap.getUnAbort())
            setControlMode(defaultControlMode);

        if (controlMode == ControlMode.trajectoryControl) {
            if (Math.abs(getInput()) > config.getAutoAbortThreshold())
                setControlMode(defaultControlMode);
        }

        if (config.inputMap != null && config.inputMap.getAbort())
            setControlMode(ControlMode.disabled);

        switch (controlMode) {
            case disabled:
                motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                setPower(0);
                break;

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

            case gamePadTorqueControl:
                targetTorque = getInput() * getTorqueControlSensitivity();
            case torqueControl:
                setPower(targetTorque + getStaticFeedforward(targetTorque));
                break;

            case trajectoryControl:
                if (!activeTrajectory.isActive()) {
                    setControlModeUnsafe(defaultControlMode);
                    activeTrajectory = null;
                    break;
                }
                MotionState targetMotionState = activeTrajectory.sampleTrajectory();

                //MotionState.telemetryMotionState(opMode.telemetry, targetMotionState, axisName + " target");

                setTargetPosition(targetMotionState.position);
                targetVelocity = targetMotionState.velocity;
                targetAcceleration = targetMotionState.acceleration;

                updatePositionPID(getTargetPosition(), getVelocityFeedforward() + getAccelerationFeedforward());
                break;

            case followTheLeader:
                if (leaderControlAxis == null)
                    throw new NullPointerException("use assignLeaderControlAxis to assign the leader before update is called");
                setTargetPosition(leaderControlAxis.getTargetPosition() + leaderPositionOffset);
                targetVelocity = leaderControlAxis.targetVelocity;
                targetAcceleration = leaderControlAxis.targetAcceleration;

                updatePositionPID(getTargetPosition(), getVelocityFeedforward() + getAccelerationFeedforward());
                break;

            case testing:
                break;
        }

        miscUpdate();
        debugUpdate();
        updateStopwatch.addTimeToTelemetryAndReset(opMode.telemetry, "end of Control axis update time");
    }

    public void homeAxis() {

    }
}
