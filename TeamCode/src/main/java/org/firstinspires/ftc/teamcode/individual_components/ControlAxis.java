package org.firstinspires.ftc.teamcode.individual_components;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.motionControl.CustomPID;
import org.firstinspires.ftc.teamcode.motionControl.MultiTorqueMotor;
import org.firstinspires.ftc.teamcode.motionControl.PositionDerivatives;

public abstract class ControlAxis {


    public static double staticThresholdUnitsPerSec = 0;
    public static double staticFrictionComp = 0;
    public static double kinematicFrictionComp = 0;

    public final String axisName;
    public final String unitName;

    public final double unitsPerEncoderCount;


    protected OpMode opMode;
    protected RobotConfig config;


    protected MultiTorqueMotor motors;

    protected abstract void initMotors();


    public enum ControlMode {

        disabled,
        directControl,
        directTorqueControl,
        positionControl,
        velocityControl,
        testing
    }

    ControlMode controlMode = ControlMode.disabled;

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
                targetPosition = getPosition();
                this.controlMode = ControlMode.velocityControl;
                motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;

            case positionControl:
                this.controlMode = ControlMode.positionControl;
                motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;

            case directControl:
                targetPosition = getPosition();
                this.controlMode = ControlMode.directControl;
                motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;

            default:
                this.controlMode = controlMode;
        }
    }


    protected PositionDerivatives positionDerivatives;

    protected CustomPID positionPID;

    void initPid() {
        positionPID = new CustomPID(opMode, config, axisName + " positionPID");
    }

    protected abstract void updatePositionPIDCoefficients();


    double targetPosition = 0;

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


    public ControlAxis(OpMode opMode, RobotConfig config, String axisName, String unitName, double unitsPerEncoderCount) {
        this.opMode = opMode;
        this.config = config;
        this.axisName = axisName;

        this.unitName = unitName;
        this.unitsPerEncoderCount = unitsPerEncoderCount;

        motors = new MultiTorqueMotor(opMode.hardwareMap, config.sensorData);

        initPid();
        initMotors();

        positionDerivatives = new PositionDerivatives(getPosition());


        DcMotor.RunMode runMode = motors.getMode();
        motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.setMode(runMode);

        updatePositionPIDCoefficients();
    }

    /**
     * updated position derivatives and checks for an abort
     *
     * @param deltaTime chang in time since last update
     */
    protected void updateEssentials(double deltaTime) {
        //opMode.telemetry.addData()
        positionDerivatives.update(getPosition(), deltaTime);

        adjustOffsetForPhysicalLimits();

        if (config.inputMap.getAbort())
            controlMode = ControlMode.disabled;

        if (controlMode == ControlMode.disabled) {
            motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors.setPower(0);
        }

        if (config.debugConfig.getControlModeDebug())
            opMode.telemetry.addData(axisName + "ControlMode", controlMode.name());

        if (config.debugConfig.getAllPositionDebug())
            opMode.telemetry.addData(axisName + " position " + unitName, getPosition());

    }


    protected void updatePositionPID(double targetPosition, double deltaTime, double feedforward) {
        updatePositionPIDCoefficients();

        double targetTorque = positionPID.runPID(targetPosition, getPosition(), deltaTime);
        feedforward += frictionCompensation(targetTorque, positionDerivatives.getVelocity());
        targetTorque += feedforward;

        motors.setTorque(targetTorque, positionDerivatives.getVelocity() / unitsPerEncoderCount);
    }

    /**
     * returns the current position of the axis in the designated units
     */
    public double getPosition() {
        return motors.getCurrentPosition() * unitsPerEncoderCount + positionOffset;
    }

    double frictionCompensation(double targetDirection, double velocity) {
        if (Math.abs(velocity) < staticThresholdUnitsPerSec)
            return staticFrictionComp * Math.copySign(1, -targetDirection);

        return kinematicFrictionComp * Math.copySign(1, velocity);
    }

    public double getVelocityTPS() {
        return positionDerivatives.getVelocity() / unitsPerEncoderCount;
    }


}
