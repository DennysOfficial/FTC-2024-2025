package org.firstinspires.ftc.teamcode.individual_components.Pivot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;

@Config
public class PivotAdvanced extends PivotBasic {

    public static double liftCGRadiusRetracted = 5;
    public static double liftMovingMassProportion = 0.35f;
    public static double liftMass = 0.004;
    public static double staticFrictionTorque = 0;
    public static double kinematicFrictionTorque = 0;
    public static double staticThresholdDPS = 10;
    public static double maxMotorSpeedRPM = 6000;
    public static double maxMotorAmps;

    public static boolean advancedDebugEnabled = false;

    public enum controlMode {
        positionControl,
        velocityControl,
    }

    public PivotAdvanced(LinearOpMode opMode, RobotConfig config) {
        super(opMode, config);
    }

    public void directControlFancy(double liftExtension) {
        setTorqueFeedforwardCompensated(config.getPivotStick(),liftExtension);
    }

    @Override
    public void setTargetVelocity(double targetVelocity) { //TODO

    }

    public void setTorque(double targetTorque) {
        double motorSpeedRPM = getVelocityTPS()/encoderCountsPerRevMotor;
        double motorPower = targetTorque + motorSpeedRPM/maxMotorSpeedRPM;

        if(advancedDebugEnabled){
            opMode.telemetry.addData("targetTorque", targetTorque);
            opMode.telemetry.addData("motorCurrent",getCurrentAmp());
        }

        setPower(motorPower);
    }

    public void setTorqueFeedforwardCompensated(double targetPower, double liftExtension) {
        double outputTorque = targetPower;
        double gravityTorque = calculateTorqueGravity(liftCGRadiusRetracted + liftExtension * liftMovingMassProportion, getAngle(), liftMass);
        double frictionTorque = calculateTorqueFriction(targetPower, getVelocityDPS(), staticFrictionTorque, kinematicFrictionTorque, staticThresholdDPS);
        outputTorque -= gravityTorque;
        outputTorque -= frictionTorque;
        setTorque(outputTorque);

        if (advancedDebugEnabled){
            opMode.telemetry.addData("gravityTorque", gravityTorque);
            opMode.telemetry.addData("frictionTorque",frictionTorque);
        }
    }

    double calculateTorqueGravity(double liftCGRadius, double pivotAngleDeg, double liftMass) {
        return liftCGRadius * Math.sin(Math.toRadians(pivotAngleDeg)) * liftMass;
    }

    double calculateTorqueFriction(double targetDirection, double pivotVelocity, double staticFrictionTorque, double kinematicFrictionTorque, double staticThreshold) {
        if (Math.abs(pivotVelocity) < staticThreshold)
            return staticFrictionTorque * Math.copySign(1, -targetDirection);

        return kinematicFrictionTorque * Math.copySign(1, pivotVelocity);
    }

    double calculateTorqueAcceleration(double liftCGRadius, double targetAcceleration) {
        return liftCGRadius * liftCGRadius * targetAcceleration;
    }

    /**
     * @return average current between the two motors in amps
     */
    double getCurrentAmp(){
       return (pivotMotorL.getCurrent(CurrentUnit.AMPS) + pivotMotorR.getCurrent(CurrentUnit.AMPS))/2.0;
    }

    public void updatePID(double deltaTime) {

    }
}
