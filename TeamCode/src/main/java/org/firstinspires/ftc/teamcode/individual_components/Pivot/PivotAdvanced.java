package org.firstinspires.ftc.teamcode.individual_components.Pivot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;

@Config
public class PivotAdvanced extends PivotBasic {

    public static double liftCGRadiusRetracted = 5;
    public static double liftMovingMassProportion = 0.5f;
    public static double liftMass = 0;
    public static double staticFrictionTorque = 0;
    public static double kinematicFrictionTorque = 0;
    public static double staticThresholdDPS = 10;

    public enum controlMode {
        positionControl,
        velocityControl,
    }

    public PivotAdvanced(LinearOpMode opMode, RobotConfig config) {
        super(opMode, config);
    }

    PIDFCoefficients pidfCoefficients = new PIDFCoefficients();

    public void directControlFancy(double liftExtension) {
        setTorqueFeedforwardCompensated(config.getPivotStick(),liftExtension);
    }

    @Override
    public void setTargetVelocity(double targetVelocity) {

    }

    public void setTorque(double targetTorque) { //TODO
        setPower(targetTorque);
    }

    public void setTorqueFeedforwardCompensated(double targetPower, double liftExtension) {
        double torque = targetPower;
        torque -= calculateTorqueGravity(liftCGRadiusRetracted + liftExtension * liftMovingMassProportion, getAngle(), liftMass);
        torque -= calculateTorqueFriction(targetPower, getVelocityDPS(), staticFrictionTorque, 0, staticThresholdDPS);
        setTorque(torque);
    }

    double calculateTorqueGravity(double liftCGRadius, double pivotAngleDeg, double liftMass) {
        return liftCGRadius * Math.sin(Math.toRadians(pivotAngleDeg)) * liftMass;
    }

    double calculateTorqueFriction(double targetDirection, double pivotVelocity, double staticFrictionTorque, double kinematicFrictionTorque, double staticThreshold) {
        if (Math.abs(pivotVelocity) < staticThreshold)
            return Math.copySign(staticFrictionTorque, -targetDirection);

        return Math.copySign(kinematicFrictionTorque, -pivotVelocity);
    }

    double calculateTorqueAcceleration(double liftCGRadius, double targetAcceleration) {
        return liftCGRadius * liftCGRadius * targetAcceleration;
    }

    public void updatePID(double deltaTime) {

    }
}
