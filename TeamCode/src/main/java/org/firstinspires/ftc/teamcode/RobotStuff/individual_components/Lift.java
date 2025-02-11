package org.firstinspires.ftc.teamcode.RobotStuff.individual_components;

import android.util.Range;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ReadOnlyRuntime;

@Config
public class Lift extends ControlAxis {

    Pivot pivot;

    public void assignPivot(Pivot pivot) {
        if (pivot == null)
            throw new NullPointerException("the pivot you tried to assign is null you goober");
        this.pivot = pivot;
    }

    public static double gCompMultiplier = 0.1;

    public static double Kp = 0.8;
    public static double Ki = 0.02;
    public static double Kd = 0.03;

    public static double staticFrictionCoefficient = 0;
    public static double kineticFrictionCoefficient = 0;
    public static double staticThreshold = 0.1;

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
    protected void initMotors() {
        motors.addMotor(config.deviceConfig.rightLift, DcMotorSimple.Direction.FORWARD);
        // motors.addMotor(config.deviceConfig.leftLift, DcMotorSimple.Direction.REVERSE);

        //motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors.getMotor(config.deviceConfig.leftLift).setMotorDisable();

    }

    @Override
    double getStaticFeedforward(double targetDirection) {
        if (pivot == null)
            throw new NullPointerException("run the assign pivot method before running anything else");

        return staticFrictionForce(targetDirection, staticFrictionCoefficient, staticThreshold) - Math.cos(Math.toRadians(pivot.getPosition())) * gCompMultiplier;
    }

    @Override
    double getVelocityFeedforward() {
        return kineticFrictionForce(targetVelocity, kineticFrictionCoefficient, staticThreshold) + targetVelocity * velocityFeedforwardCoefficient;
    }

    @Override
    double getAccelerationFeedforward() {
        return 0;
    }


    public Lift(ControlMode defaultControlMode, OpMode opMode, RobotConfig config) {
        super(defaultControlMode, opMode, config, "Lift", "inches", 27.0 / 4300.0);

        softLimits = new Range<>(0.5, 31.0);

        physicalLimits = new Range<>(0.0, Double.POSITIVE_INFINITY);
    }

    double previousTargetPivotPosition = Double.NaN;
    @Override
    public void setTargetPosition(double targetPosition) {
        if (targetPosition == getTargetPosition() && previousTargetPivotPosition == (previousTargetPivotPosition = pivot.getTargetPosition()))
            return;

        if (pivot == null)
            throw new NullPointerException("run the assign pivot method before setting target position");

        double dynamicUpperLimit = config.getFrontExtensionLimitInch() / Math.sin(Math.toRadians(pivot.getTargetPosition())) - config.getRetractedLiftLengthInch();
        dynamicUpperLimit = Math.abs(dynamicUpperLimit);
        targetPosition = MathUtils.clamp(targetPosition, Double.NEGATIVE_INFINITY, dynamicUpperLimit);

        //opMode.telemetry.addData("liftDynamicLimit", upperLimit);

        super.setTargetPosition(targetPosition);
    }

    @Override
    void miscUpdate() {

    }

}
