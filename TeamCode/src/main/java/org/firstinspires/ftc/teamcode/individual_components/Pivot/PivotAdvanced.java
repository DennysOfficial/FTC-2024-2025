package org.firstinspires.ftc.teamcode.individual_components.Pivot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;
import org.firstinspires.ftc.teamcode.MathStuff;
import org.firstinspires.ftc.teamcode.motionControl.CustomPID;
import org.firstinspires.ftc.teamcode.motionControl.PositionDerivatives;

@Config
public class PivotAdvanced extends PivotBasic {


    public static double staticThresholdDPS = 2;


    @Config
    public static class coefficients {
        public static double staticFrictionTorque = 0;
        public static double kinematicFrictionTorque = 0;
        public static double directControlDamping = 0;
    }

    @Config
    public static class liftProperties {
        public static double extendedGComp = .4;
        public static double retractedGComp = .11;
        public static double maxExtension = 27.3;
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
        PIDControl,
        testing
    }

    public ControlSate controlSate = ControlSate.directControl;

    CustomPID pivotPID;

    PositionDerivatives positionDerivatives; // calculates velocity and acceleration

    public PivotAdvanced(LinearOpMode opMode, RobotConfig config) {
        super(opMode, config);
        batteryVoltageSensor = getBatteryVoltageSensor();
        pivotPID = new CustomPID(opMode, config);
        pivotPID.debugEnabled = true;
        positionDerivatives = new PositionDerivatives(getAngle());
    }


    public void directControlFancy(double liftExtensionInch) {
        setTorque(calculateNetTorque(config.getPivotStick() * config.getPivotSensitivity(), liftExtensionInch) - calculateDragTorque(positionDerivatives.getVelocity(), coefficients.directControlDamping));
        opMode.telemetry.addData("input", config.getPivotStick() * config.getPivotSensitivity());
    }

    public void update(double deltaTime, double liftExtensionInch) {

        if (config.getAbort())
            controlSate = ControlSate.directControl;

        if (config.getPivotStick() > 0.5f + config.getAutoAbortThreshold() || config.getPivotStick() < 0.5f - config.getAutoAbortThreshold())
            controlSate = ControlSate.directControl;

        switch (controlSate) {
            case directControl:
                directControlFancy(liftExtensionInch);
                break;
            case PIDControl:
                pivotPID.setCoefficients(pivotPIDCon.kP, pivotPIDCon.kI, pivotPIDCon.kD);
                setTorque(calculateNetTorque(pivotPID.runPID(targetAngle, getAngle(), deltaTime, positionDerivatives.getVelocity()), liftExtensionInch));
                break;
            case testing:

                break;
        }



    }


    @Override
    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
        controlSate = ControlSate.PIDControl;
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

        double gravityTorque = calculateTorqueGravity(liftExtension, getAngle(), liftProperties.maxExtension);
        double frictionTorque = calculateTorqueFriction(targetNetTorque, getVelocityDPS(), coefficients.staticFrictionTorque, coefficients.kinematicFrictionTorque, staticThresholdDPS);

        double outputTorque = targetNetTorque - gravityTorque - frictionTorque;

        if (advancedDebugEnabled) {
            opMode.telemetry.addData("gravityTorque", gravityTorque);
            opMode.telemetry.addData("frictionTorque", frictionTorque);
        }
        return outputTorque;
    }

    double calculateTorqueGravity(double liftExtension, double pivotAngleDeg, double maxLiftExtension) {


        return Math.sin(Math.toRadians(pivotAngleDeg)) * MathStuff.lerp(liftProperties.retractedGComp, liftProperties.extendedGComp, liftExtension / maxLiftExtension);
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
}
