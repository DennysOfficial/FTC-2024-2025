package org.firstinspires.ftc.teamcode.individual_components.Pivot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;

@Config
public class PivotAdvanced extends PivotBasic {



    public static double staticThresholdDPS = 10;

    @Config
    public static class coefficients{
        public static double staticFrictionTorque = 0;
        public static double kinematicFrictionTorque = 0;
        public static double gravityCoefficient = 1;
        public static double directControlDamping = 0;
    }

    @Config
    public static class liftProperties {
        public static double CGRadiusRetracted = 5;
        public static double movingMassProportion = 0.35f;
        public static double mass = 0.004;
    }

    @Config
    public static class motorProperties {
        public static double maxSpeedRPM = 6000;
        public static double stallAmps;
    }

    public static boolean advancedDebugEnabled = false;

    public enum controlMode {
        positionControl,
        velocityControl,
    }

    public PivotAdvanced(LinearOpMode opMode, RobotConfig config) {
        super(opMode, config);
    }

    public void directControlFancy(double liftExtension) {
        setTorque(calculateNetTorque(config.getPivotStick(), liftExtension) - calculateDragTorque(getVelocityDPS(),coefficients.directControlDamping));
    }

    @Override
    public void setTargetVelocity(double targetVelocity) { //TODO

    }

    public void setTorque(double targetTorque) {
        double motorSpeedRPM = getVelocityTPS() / encoderCountsPerRevMotor;
        double motorPower = targetTorque + motorSpeedRPM / motorProperties.maxSpeedRPM;

        if (advancedDebugEnabled) {
            opMode.telemetry.addData("targetTorque", targetTorque);
            opMode.telemetry.addData("motorCurrent", getCurrentAmp());
        }

        setPower(motorPower);
    }

    public double calculateNetTorque(double targetNetTorque, double liftExtension) {

        double gravityTorque = calculateTorqueGravity(liftProperties.CGRadiusRetracted + liftExtension * liftProperties.movingMassProportion, getAngle(), liftProperties.mass);
        double frictionTorque = calculateTorqueFriction(targetNetTorque, getVelocityDPS(), coefficients.staticFrictionTorque, coefficients.kinematicFrictionTorque, staticThresholdDPS);

        double outputTorque = targetNetTorque - gravityTorque -frictionTorque;

        if (advancedDebugEnabled) {
            opMode.telemetry.addData("gravityTorque", gravityTorque);
            opMode.telemetry.addData("frictionTorque", frictionTorque);
        }
        return outputTorque;
    }

    double calculateTorqueGravity(double liftCGRadius, double pivotAngleDeg, double liftMass) {
        return liftCGRadius * Math.sin(Math.toRadians(pivotAngleDeg)) * liftMass * coefficients.gravityCoefficient;
    }

    double calculateTorqueFriction(double targetDirection, double pivotVelocity, double staticFrictionTorque, double kinematicFrictionTorque, double staticThreshold) {
        if (Math.abs(pivotVelocity) < staticThreshold)
            return staticFrictionTorque * Math.copySign(1, -targetDirection);

        return kinematicFrictionTorque * Math.copySign(1, pivotVelocity);
    }

    double calculateTorqueAcceleration(double liftCGRadius, double targetAcceleration) {
        return liftCGRadius * liftCGRadius * targetAcceleration;
    }

    double calculateDragTorque(double pivotVelocity, double dragCoefficient) {
        return pivotVelocity * dragCoefficient;
    }

    /**
     * @return average current between the two motors in amps
     */
    double getCurrentAmp() {
        return (pivotMotorL.getCurrent(CurrentUnit.AMPS) + pivotMotorR.getCurrent(CurrentUnit.AMPS)) / 2.0;
    }

    public void updatePID(double deltaTime) {

    }
}
