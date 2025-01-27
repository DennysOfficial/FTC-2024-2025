package org.firstinspires.ftc.teamcode.RobotStuff.individual_components;

import android.util.Range;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

@Config
<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotStuff/individual_components/LeftLift.java
public class LeftLift extends ControlAxis {

    LeftPivot leftPivot;

    final double retractedRadius = 10;

    public void assignPivot(LeftPivot leftPivot) {
        if (leftPivot == null)
            throw new NullPointerException("the pivot you tried to assign is null you goober");
        this.leftPivot = leftPivot;
========
public class OldLift extends ControlAxis {

    OldPivot oldPivot;

    public void assignPivot(OldPivot oldPivot) {
        if (oldPivot == null)
            throw new NullPointerException("the pivot you tried to assign is null you goober");
        this.oldPivot = oldPivot;
>>>>>>>> TeleOp:NewTeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotStuff/individual_components/OldLift.java
    }

    public static double gCompMultiplier = 0.1;

    public static double Kp = 0.5;
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
        motors.addMotor(config.deviceConfig.rightLift, DcMotorSimple.Direction.FORWARD);

        motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    double getStaticFeedforward(double targetDirection) {
<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotStuff/individual_components/LeftLift.java
        if (leftPivot == null)
            throw new NullPointerException("run the assign pivot method before running anything else");

        return staticFrictionForce(targetDirection, staticFrictionCoefficient, staticThreshold) + Math.cos(Math.toRadians(leftPivot.getPosition())) * gCompMultiplier;
========
        if (oldPivot == null)
            throw new NullPointerException("run the assign pivot method before running anything else");

        return staticFrictionForce(targetDirection, staticFrictionCoefficient, staticThreshold) - Math.cos(Math.toRadians(oldPivot.getPosition())) * gCompMultiplier;
>>>>>>>> TeleOp:NewTeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotStuff/individual_components/OldLift.java
    }

    @Override
    double getVelocityFeedforward() {
        return kineticFrictionForce(targetVelocity, kineticFrictionCoefficient, staticThreshold) + targetVelocity * velocityFeedforwardCoefficient;
    }

    @Override
    double getAccelerationFeedforward() {
        return 0;
    }


<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotStuff/individual_components/LeftLift.java
    public LeftLift(ControlMode defaultControlMode, OpMode opMode, RobotConfig config) {
        super(defaultControlMode, opMode, config, "LeftLift", "inches", (19.25-55)/(-44-2560));
========
    public OldLift(ControlMode defaultControlMode, OpMode opMode, RobotConfig config) {
        super(defaultControlMode, opMode, config, "Lift", "inches", 27.0 / 4300.0);
>>>>>>>> TeleOp:NewTeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotStuff/individual_components/OldLift.java

        softLimits = new Range<>(0.5, 34.69);

        physicalLimits = new Range<>(0.0, Double.POSITIVE_INFINITY);
    }

<<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotStuff/individual_components/LeftLift.java
    double previousRightPivotTargetPosition = Double.NaN;
    @Override
    public void setTargetPosition(double targetPosition) {
        if (leftPivot == null)
            throw new NullPointerException("run the assign pivot method before setting target position");

        if (targetPosition == getTargetPosition() && previousRightPivotTargetPosition == (previousRightPivotTargetPosition = leftPivot.getTargetPosition()))
            return;

        double dynamicUpperLimit = config.getFrontExtensionLimitInch() / Math.sin(Math.toRadians(leftPivot.getTargetPosition())) - retractedRadius;
========
    @Override
    public void setTargetPosition(double targetPosition) {
        if (targetPosition == getTargetPosition())
            return;

        if (oldPivot == null)
            throw new NullPointerException("run the assign pivot method before setting target position");

        double dynamicUpperLimit = config.getFrontExtensionLimitInch() / Math.sin(Math.toRadians(oldPivot.getPosition())) - config.getRetractedLiftLengthInch();
>>>>>>>> TeleOp:NewTeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotStuff/individual_components/OldLift.java
        dynamicUpperLimit = Math.abs(dynamicUpperLimit);
        targetPosition = MathUtils.clamp(targetPosition, Double.NEGATIVE_INFINITY, dynamicUpperLimit);

        //opMode.telemetry.addData("liftDynamicLimit", upperLimit);

        super.setTargetPosition(targetPosition);
    }

    @Override
    void miscUpdate() {

    }

}
