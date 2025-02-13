package org.firstinspires.ftc.teamcode.RobotStuff.individual_components;

import android.util.Range;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

@Config
public class LeftLift extends ControlAxis {

    LeftPivot leftPivot;

    DigitalChannel limitSwitch;
    public static double homingPosition = 0;
    public static double homingPower = 0.5;


    final double retractedRadius = 10;

    public void assignPivot(LeftPivot leftPivot) {
        if (leftPivot == null)
            throw new NullPointerException("the pivot you tried to assign is null you goober");
        this.leftPivot = leftPivot;
    }

    public static double gCompMultiplier = -0.069;

    public static double Kp = -2;
    public static double Ki = 0;
    public static double Kd = -0.03;

    public static double staticFrictionCoefficient = 0;
    public static double kineticFrictionCoefficient = 0;
    public static double staticThreshold = 0.1;


    @Override
    public void homeAxis() {
        setControlMode(ControlMode.torqueControl);
        //motors.getCurrentAlert()
        targetTorque = homingPower;

    }

    @Override
    double getKp() {
        return Kp;
    }

    @Override
    double getKi() {
        return Ki;
    }

    @Override
    double getKd() {
        return Kd;
    }

    public static double velocityFeedforwardCoefficient = 0;


    @Override
    float getInput() {
        return (config.inputMap == null) ? 0 : (float) config.inputMap.getRightLiftStick();
    }

    @Override
    float getVelocityControlMaxRate() {
        return config.sensitivities.getLiftRate();
    }

    @Override
    float getTorqueControlSensitivity() {
        return config.sensitivities.getLiftSensitivity();
    }

    @Override
    protected void initMotors() {
        motors.addMotor(config.deviceConfig.leftLift, DcMotorSimple.Direction.FORWARD);

        motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    double getStaticFeedforward(double targetDirection) {
        if (leftPivot == null)
            throw new NullPointerException("run the assign pivot method before running anything else");

        return staticFrictionForce(targetDirection, staticFrictionCoefficient, staticThreshold) + Math.cos(Math.toRadians(leftPivot.getPosition())) * gCompMultiplier;
    }

    @Override
    double getVelocityFeedforward() {
        return kineticFrictionForce(targetVelocity, kineticFrictionCoefficient, staticThreshold) + targetVelocity * velocityFeedforwardCoefficient;
    }

    @Override
    double getAccelerationFeedforward() {
        return 0;
    }


    public LeftLift(ControlMode defaultControlMode, OpMode opMode, RobotConfig config) {
        super(defaultControlMode, opMode, config, "LeftLift", "inches", 25.25 / 4056);
        limitSwitch = opMode.hardwareMap.get(DigitalChannel.class, "Left Limit Switch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        softLimits = new Range<>(0.5, 20.0);

        physicalLimits = new Range<>(0.0, 25.25);
    }

    double previousRightPivotTargetPosition = Double.NaN;

    @Override
    public void setTargetPosition(double targetPosition) {
        if (leftPivot == null)
            throw new NullPointerException("run the assign pivot method before setting target position");

        if (targetPosition == getTargetPosition() && previousRightPivotTargetPosition == (previousRightPivotTargetPosition = leftPivot.getTargetPosition()))
            return;

        double dynamicUpperLimit = config.getFrontExtensionLimitInch() / Math.sin(Math.toRadians(leftPivot.getTargetPosition())) - retractedRadius;
        dynamicUpperLimit = Math.abs(dynamicUpperLimit);
        targetPosition = MathUtils.clamp(targetPosition, Double.NEGATIVE_INFINITY, dynamicUpperLimit);

        //opMode.telemetry.addData("liftDynamicLimit", upperLimit);

        super.setTargetPosition(targetPosition);
    }

    boolean previousButtonState = false;
    @Override
    void miscUpdate() {



        //opMode.telemetry.addLine("misc update running");

        if (config.inputMap.gamepad1.y) {
            targetTorque = homingPower;
            setControlModeUnsafe(ControlMode.torqueControl);
        } else
            setControlMode(defaultControlMode);

        if(previousButtonState && !config.inputMap.gamepad1.y){
            opMode.telemetry.addData("position offset = %f", positionOffset);
            setCurrentPosition(homingPosition);
        }

        previousButtonState = config.inputMap.gamepad1.y;
    }

}
