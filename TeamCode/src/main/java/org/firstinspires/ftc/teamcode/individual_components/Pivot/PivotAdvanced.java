package org.firstinspires.ftc.teamcode.individual_components.Pivot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;
import org.firstinspires.ftc.teamcode.motionControl.CustomPID;

@Config
public class PivotAdvanced extends PivotBasic {


    public static double staticThresholdDPS = 10;


    @Config
    public static class coefficients {
        public static double staticFrictionTorque = 0;
        public static double kinematicFrictionTorque = 0;
        public static double gravityCoefficient = 1;
        public static double directControlDamping = 0;
        public static double beans = 1.0;
    }

    //@Config
    public static class liftProperties {
        public static double CGRadiusRetracted = 5;
        public static double movingMassProportion = 0.35f;
        public static double mass = 0.004;
    }

    //@Config
    public static class motorProperties {
        public static double maxSpeedRPM = 6000;
        public static double stallAmps;
    }
    @Config
    public static class pivotPIDCon {
        public static double kP = 0.05;
        public static double kI = 0;
        public static double kD = 0.002;
    }

    public static boolean advancedDebugEnabled = true;

    public enum ControlSate {
        directControl,
        PIDControl
    }

    public ControlSate controlSate = ControlSate.directControl;

    CustomPID pivotPID;

    public PivotAdvanced(LinearOpMode opMode, RobotConfig config) {
        super(opMode, config);
        batteryVoltageSensor = getBatteryVoltageSensor();
        pivotPID = new CustomPID(opMode, config);
        pivotPID.debugEnabled = true;
    }

    public void directControlFancy(double liftExtensionInch) {
        setTorque(calculateNetTorque(config.getPivotStick() * config.getPivotSensitivity(), liftExtensionInch) - calculateDragTorque(getVelocityDPS(), coefficients.directControlDamping));
    }

    public void update(double deltaTime,double liftExtensionInch){
        switch (controlSate){
            case directControl:
                directControlFancy(liftExtensionInch);
                break;
            case PIDControl:
                pivotPID.setCoefficients(pivotPIDCon.kP,pivotPIDCon.kI,pivotPIDCon.kD);
                setTorque(calculateNetTorque(pivotPID.runPID(targetAngle,getAngle(),deltaTime),liftExtensionInch));
                break;
        }

        pivotPID.setPreviousActualPosition(getAngle());
    }

    @Override
    public void setTargetAngle(double targetAngle) { //TODO
        this.targetAngle = targetAngle;
    }

    public VoltageSensor getBatteryVoltageSensor() {
        for (VoltageSensor sensor : opMode.hardwareMap.voltageSensor) {
            if (sensor.getVoltage() > 0)
                return sensor;
        }
        return null;
    }

    VoltageSensor batteryVoltageSensor;

    public void setTorque(double targetTorque) {

        double motorSpeedRPM = getVelocityTPS() / encoderCountsPerRevMotor;

        double battery = batteryVoltageSensor.getVoltage() / 12.0;

        setPower((targetTorque + motorSpeedRPM / motorProperties.maxSpeedRPM) / battery);

        if (advancedDebugEnabled) {
            opMode.telemetry.addData("pivotAngle", getAngle());
            //opMode.telemetry.addData("targetTorque", targetTorque);
            //opMode.telemetry.addData("motorCurrent", getCurrentAmp());
            //opMode.telemetry.addData("motorSpeed", motorSpeedRPM);
            opMode.telemetry.addData("motorPower", pivotMotorR.getPower());
            //opMode.telemetry.addData("batteryVoltage", batteryVoltageSensor.getVoltage());
        }
    }

    public double calculateNetTorque(double targetNetTorque, double liftExtension) {

        double gravityTorque = calculateTorqueGravity(liftProperties.CGRadiusRetracted + liftExtension * liftProperties.movingMassProportion, getAngle(), liftProperties.mass);
        double frictionTorque = calculateTorqueFriction(targetNetTorque, getVelocityDPS(), coefficients.staticFrictionTorque, coefficients.kinematicFrictionTorque, staticThresholdDPS);

        double outputTorque = targetNetTorque - gravityTorque - frictionTorque;

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
