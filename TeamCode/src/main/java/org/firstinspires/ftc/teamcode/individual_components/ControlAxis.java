package org.firstinspires.ftc.teamcode.individual_components;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.motionControl.CustomPID;
import org.firstinspires.ftc.teamcode.motionControl.MultiTorqueMotor;
import org.firstinspires.ftc.teamcode.motionControl.PositionDerivatives;

public abstract class ControlAxis {


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
        positionControl,
        velocityControl,
        testing
    }

    protected ControlMode controlMode = ControlMode.disabled;

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


    double targetPosition;

    public double getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = MathUtils.clamp(targetPosition, upperLimit, lowerLimit);
    }

    protected double upperLimit = Double.POSITIVE_INFINITY;
    protected double lowerLimit = Double.NEGATIVE_INFINITY;
    protected double positionOffset = 0;


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

        positionDerivatives = new PositionDerivatives(getPosition());

        initPid();
        initMotors();
        updatePositionPIDCoefficients();
    }
 
    /**
     * updated position derivatives and checks for an abort
     *
     * @param deltaTime chang in time since last update
     */
    protected void updateEssentials(double deltaTime) {
        positionDerivatives.update(getPosition(), deltaTime);

        if (config.inputMap.getAbort())
            controlMode = ControlMode.disabled;

        if (controlMode == ControlMode.disabled) {
            motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors.setPower(0);
        }
    }


    protected void updatePositionPID(double targetPosition, double deltaTime, double feedforward) {
        updatePositionPIDCoefficients();
        motors.setTorque(feedforward + positionPID.runPID(targetPosition, getPosition(), deltaTime, positionDerivatives.getVelocity()), positionDerivatives.getVelocity());
    }

    public double getPosition() {
        return motors.getCurrentPosition() * unitsPerEncoderCount + positionOffset;
    }


}
