package org.firstinspires.ftc.teamcode.individual_components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;
@Config
public class NewLift extends ControlAxis {


    public static double gCompMultiplier = 0.1;

    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;

    public static double velocityFeedforwardCoefficient = 0;


    @Override
    protected void initMotors() {
        motors.addMotor(config.deviceConfig.rightLift, DcMotorSimple.Direction.FORWARD);
       // motors.addMotor(config.deviceConfig.leftLift, DcMotorSimple.Direction.REVERSE);

        motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       motors.getMotor(config.deviceConfig.leftLift).setMotorDisable();
    }

    @Override
    protected void updatePositionPIDCoefficients() {
        positionPID.setCoefficients(Kp, Ki, Kd);
    }


    public NewLift(OpMode opMode, RobotConfig config) {
        super(opMode, config, "Lift", "inches", 27.0 / 4300.0);

        upperLimit = 31;
        lowerLimit = 0.2;

        physicalUpperLimit = 33.75;;
        physicalLowerLimit = 0;
    }


    public void update(double deltaTime, double pivotAngleDeg) {
        updateEssentials(deltaTime);

        switch (controlMode) {
            case directControl:
                targetVelocity = config.inputMap.getLiftStick() * config.sensitivities.getLiftRate();

                setTargetPosition(getPosition() + targetVelocity * deltaTime);
                double directFeedforward = -calcGravityForce(pivotAngleDeg) + targetVelocity * velocityFeedforwardCoefficient;
                updatePositionPID(targetPosition, deltaTime, directFeedforward);
                break;

            case positionControl:
                double positionFeedforward = -calcGravityForce(pivotAngleDeg);
                updatePositionPID(targetPosition, deltaTime, positionFeedforward);
                break;

            case velocityControl:
                setTargetPosition(getPosition() + targetVelocity * deltaTime);
                double velocityFeedforward = -calcGravityForce(pivotAngleDeg) + targetVelocity * velocityFeedforwardCoefficient;
                updatePositionPID(targetPosition, deltaTime, velocityFeedforward);
                break;

            case directTorqueControl:
                motors.setTorque(config.inputMap.getLiftStick() * config.sensitivities.getLiftSensitivity() - calcGravityForce(pivotAngleDeg), getVelocityTPS());
                break;

            case testing:

        }
    }

    public double calcGravityForce(double pivotAngleDeg) {
        return -Math.cos(Math.toRadians(pivotAngleDeg)) * gCompMultiplier;
    }
}
