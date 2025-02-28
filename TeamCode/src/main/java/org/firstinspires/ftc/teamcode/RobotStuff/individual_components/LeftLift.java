package org.firstinspires.ftc.teamcode.RobotStuff.individual_components;

import android.util.Range;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ButtonEdgeDetector;

@Config
public class LeftLift extends ControlAxis {

    LeftPivot leftPivot;

    DigitalChannel limitSwitch;
    public static double homingPosition = 0;
    public static double homingRetractPower = 0.5;
    public static double homingDwellPower = 0.2;
    public static double homingDwellPeriod = 0.3;


    final double retractedRadius = 10;

    public void assignPivot(LeftPivot leftPivot) {
        if (leftPivot == null)
            throw new NullPointerException("the pivot you tried to assign is null you goober");
        this.leftPivot = leftPivot;
    }

    public static double gCompMultiplier = 0.069;

    public static double Kp = 3;
    public static double Ki = 0;
    public static double Kd = 0.03;

    public static double staticFrictionCoefficient = 0;
    public static double kineticFrictionCoefficient = 0;
    public static double staticThreshold = 0.1;


    @Override
    public void homeAxis() {
        homingState = HomingState.homed;
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
        return (config.inputMap == null) ? 0 : (float) config.inputMap.getLiftStick();
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
    protected void addMotors() {
        motors.addMotor(config.deviceConfig.leftLift, DcMotorSimple.Direction.REVERSE);

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
        super(defaultControlMode, opMode, config, "LeftLift", "inches", 25.25 / 4056.0);
        limitSwitch = opMode.hardwareMap.get(DigitalChannel.class, "Left Limit Switch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        softLimits = new Range<>(0.5, 25.420);

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
    double minExtension = Double.POSITIVE_INFINITY;

    ButtonEdgeDetector homingButton = new ButtonEdgeDetector(false);

    ElapsedTime homingDwellTimer;

    @Override
    void miscUpdate() {

        if (homingButton.getButtonDown(config.inputMap.gamepad1.dpad_down)) {
            homeAxis();
        }

        opMode.telemetry.addData("limit switch state:", limitSwitch.getState());

        switch (homingState) {
            case initHoming:
                minExtension = Double.POSITIVE_INFINITY;
                homingState = HomingState.retracting;

                setControlMode(ControlMode.torqueControl);
                break;

            case retracting:
                targetTorque = homingRetractPower;

                minExtension = Math.min(getPosition(), minExtension);

                if (!limitSwitch.getState()) {
                    homingDwellTimer = new ElapsedTime();
                    homingState = HomingState.dwelling;
                }
                break;

            case dwelling:
                targetTorque = homingDwellPower;

                minExtension = Math.min(getPosition(), minExtension);

                if (homingDwellTimer.seconds() > homingDwellPeriod) {
                    homingState = HomingState.finishingHoming;
                }
                break;

            case finishingHoming:
                positionOffset -= minExtension - homingPosition;
                setControlMode(defaultControlMode);
                homingState = HomingState.homed;
                break;

            case homed:
                //congratulations
        }

    }
}
