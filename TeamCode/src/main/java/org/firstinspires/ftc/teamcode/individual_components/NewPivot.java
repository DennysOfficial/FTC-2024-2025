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

    static final double maxLiftExtension = 27;
    public static double extendedGComp = .2;
    public static double retractedGComp = .05;

    public static double posKp = 0;
    public static double posKi = 0;
    public static double posKd = 0;

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
    }


    public void update(double deltaTime, double liftExtension) {
        updateEssentials(deltaTime);

        switch (controlMode) {
            case directControl:
                targetVelocity = config.inputMap.getLiftStick() * config.sensitivities.getLiftRate();

                setTargetPosition(getPosition() + targetVelocity * deltaTime);
                double directFeedforward = -calculateTorqueGravity(liftExtension) + targetVelocity * velocityFeedforwardCoefficient;
                updatePositionPID(targetPosition, deltaTime, directFeedforward);
                break;

            case positionControl:
                double positionFeedforward = -calculateTorqueGravity(liftExtension);
                updatePositionPID(targetPosition, deltaTime, positionFeedforward);
                break;

            case velocityControl:
                setTargetPosition(getPosition() + targetVelocity * deltaTime);
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
        return Math.sin(Math.toRadians(getPosition())) * MathStuff.lerp(extendedGComp, retractedGComp, liftExtension / maxLiftExtension);
    }
}
