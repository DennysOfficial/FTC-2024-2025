package org.firstinspires.ftc.teamcode.individual_components.Pivot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;


public class PivotAdvanced extends PivotBasic {

    public enum controlMode {
        positionControl,
        velocityControl,
    }

    public PivotAdvanced(LinearOpMode opMode, RobotConfig config) {
        super(opMode, config);
    }

    PIDFCoefficients pidfCoefficients = new PIDFCoefficients();

    public void directControlCustomPID() {
        setTargetVelocity(config.getPivotStick() * config.getPivotRate());
    }

    @Override
    public void setTargetVelocity(double targetVelocity) {

    }

    double calculateFeedforwardGravity(double liftCGRadius, double pivotAngleDeg) {
        return liftCGRadius * Math.sin(Math.toRadians(pivotAngleDeg));
    }

    double calculateFeedforwardFriction(double pivotVelocity, double staticFrictionTorque, double kinematicFrictionTorque, double staticThreshold) {
        if (Math.abs(pivotVelocity) < staticThreshold)
            return Math.copySign(staticFrictionTorque, -pivotVelocity);
        return Math.copySign(kinematicFrictionTorque, -pivotVelocity);
    }

    public void updatePID(double deltaTime) {

    }
}
