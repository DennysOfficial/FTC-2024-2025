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

    PositionDerivatives positionDerivatives; // calculates velocity, acceleration, and jerk

    public PivotAdvanced(LinearOpMode opMode, RobotConfig config) {
        super(opMode, config);
        pivotPID = new CustomPID(opMode, config);
        pivotPID.debugEnabled = true;
        positionDerivatives = new PositionDerivatives(getAngle());
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

        if(config.debugConfig.isPivotPositionAndDerivativesDebug()){
            opMode.telemetry.addData("pivot angle", getAngle());
            opMode.telemetry.addData("pivot velocity", positionDerivatives.getVelocity());
            opMode.telemetry.addData("pivot acceleration", positionDerivatives.getAcceleration());
            opMode.telemetry.addData("pivot jerk", positionDerivatives.getJerk());
        }
    }

    public void directControlFancy(double liftExtensionInch) {
        double dampingTorque = calculateDragTorque(positionDerivatives.getVelocity(), coefficients.directControlDamping);
        setTorque(calculateNetTorque(config.getPivotStick() * config.getPivotSensitivity(), liftExtensionInch) - dampingTorque);

        opMode.telemetry.addData("dampingTorque", dampingTorque);
    }

    @Override
    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
        controlSate = ControlSate.PIDControl;
    }

    public void setTorque(double targetTorque) {

        double motorSpeedRPM = getVelocityTPS() / encoderCountsPerRevMotor;

        double battery = config.batteryVoltageSensor.getVoltage() / 12.0;

        setPower((targetTorque + motorSpeedRPM / motorProperties.maxSpeedRPM) / battery);

        if (advancedDebugEnabled) {
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
        pivotAngleDeg = Math.toRadians(pivotAngleDeg);
        return Math.sin(pivotAngleDeg) * MathStuff.lerp(liftProperties.retractedGComp, liftProperties.extendedGComp, liftExtension / maxLiftExtension);
    }

    double calculateTorqueFriction(double targetDirection, double pivotVelocity, double staticFrictionTorque, double kinematicFrictionTorque, double staticThreshold) {
        if (Math.abs(pivotVelocity) < staticThreshold)
            return staticFrictionTorque * Math.copySign(1, -targetDirection);

        return kinematicFrictionTorque * Math.copySign(1, pivotVelocity);
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
