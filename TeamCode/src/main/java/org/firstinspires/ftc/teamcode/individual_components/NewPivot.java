package org.firstinspires.ftc.teamcode.individual_components;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.MathStuff;
import org.firstinspires.ftc.teamcode.individual_components.Pivot.PivotAdvanced;

@Config
public class NewPivot extends ControlAxis {


    static final int encoderCountsPerRevMotor = 28;
    static final double finalGearRatio = 1. / 200.; // rotations of final over rotations of motor
    static final double encoderCountsPerRevFinal = encoderCountsPerRevMotor / finalGearRatio;
    static final double encoderCountsPerDeg = encoderCountsPerRevFinal / 360;

    static final double maxLiftExtension = 27 + 27.0 / 4.0;
    public static double extendedGComp = 0.2;
    public static double retractedGComp = 0.12;

    public static double posKp = 0.1;
    public static double posKi = 0.01;
    public static double posKd = 0.005;

    public static double velocityFeedforwardCoefficient = 0;


    @Override
    protected void initMotors() {
        motors.addMotor(config.deviceConfig.leftPivot, DcMotorSimple.Direction.FORWARD);
        motors.addMotor(config.deviceConfig.rightPivot, DcMotorSimple.Direction.REVERSE);

        motors.setTargetPosition(0);
        motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    protected void updatePositionPIDCoefficients() {
        positionPID.setCoefficients(posKp, posKi, posKd);
    }


    public NewPivot(OpMode opMode, RobotConfig config) {
        super(opMode, config, "Pivot", "Degrees", 1.0 / encoderCountsPerDeg);

        lowerLimit = -40;
        upperLimit = 86.9;
    }


    public void update(double deltaTime, double liftExtension) {
        updateEssentials(deltaTime);

        if (config.debugConfig.pivotTorqueDebug()) {
            opMode.telemetry.addData("Pivot gravity", calculateTorqueGravity(liftExtension));
        }

        switch (controlMode) {
            case directControl:
                targetVelocity = config.inputMap.getPivotStick() * config.sensitivities.getPivotRate();

                setTargetPosition(getTargetPosition() + targetVelocity * deltaTime);
                double directFeedforward = -calculateTorqueGravity(liftExtension) + targetVelocity * velocityFeedforwardCoefficient;
                updatePositionPID(targetPosition, deltaTime, directFeedforward);
                break;

            case positionControl:
                double positionFeedforward = -calculateTorqueGravity(liftExtension);
                updatePositionPID(targetPosition, deltaTime, positionFeedforward);
                break;

            case velocityControl:
                setTargetPosition(getTargetPosition() + targetVelocity * deltaTime);
                double velocityFeedforward = -calculateTorqueGravity(liftExtension) + targetVelocity * velocityFeedforwardCoefficient;
                updatePositionPID(targetPosition, deltaTime, velocityFeedforward);
                break;

            case directTorqueControl:
                motors.setTorque(config.inputMap.getPivotStick() * config.sensitivities.getPivotSensitivity() - calculateTorqueGravity(liftExtension), getVelocityTPS());
                break;

            case testing:

        }

    }

    double calculateTorqueGravity(double liftExtension) {
        double interpolationAmount = liftExtension / maxLiftExtension;
        opMode.telemetry.addData("interpolation amount", interpolationAmount);

        return Math.sin(Math.toRadians(getPosition())) * MathStuff.lerp(retractedGComp, extendedGComp, interpolationAmount);
    }
}
