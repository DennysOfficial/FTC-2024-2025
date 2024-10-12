package org.firstinspires.ftc.teamcode.individual_components.Pivot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

    @Config
    public static class pivotPIDCon {
        public static double kP = 0.05;
        public static double kI = 0;
        public static double kD = 0.002;
    }

    public enum PivotControlSate {
        directControl,
        PIDControl,
        testing
    }
    public PivotControlSate controlSate = PivotControlSate.directControl;

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
            controlSate = PivotControlSate.directControl;

        if (config.getPivotStick() > config.getAutoAbortThreshold() || config.getPivotStick() < -config.getAutoAbortThreshold())
            controlSate = PivotControlSate.directControl;

        opMode.telemetry.addData("pivotStick", config.getPivotStick());
        opMode.telemetry.addData("pivotStatus", controlSate);


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

        if (config.debugConfig.pivotPositionAndDerivativesDebug()) {
            opMode.telemetry.addData("pivot angle", getAngle());
            opMode.telemetry.addData("pivot velocity", positionDerivatives.getVelocity());
            opMode.telemetry.addData("pivot acceleration", positionDerivatives.getAcceleration());
        }
    }

    public void directControlFancy(double liftExtensionInch) {
        double dampingTorque = calculateDragTorque(positionDerivatives.getVelocity(), coefficients.directControlDamping);
        setTorque(calculateNetTorque(config.getPivotStick() * config.getPivotSensitivity(), liftExtensionInch) - dampingTorque);
        if (config.debugConfig.pivotTorqueDebug())
            opMode.telemetry.addData("dampingTorque", dampingTorque);
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
        controlSate = PivotControlSate.PIDControl;
    }


    public double calculateNetTorque(double targetNetTorque, double liftExtension) {

        double gravityTorque = calculateTorqueGravity(liftExtension, getAngle(), liftProperties.maxExtension);
        double frictionTorque = calculateTorqueFriction(targetNetTorque, getVelocityDPS(), coefficients.staticFrictionTorque, coefficients.kinematicFrictionTorque, staticThresholdDPS);

        double outputTorque = targetNetTorque - gravityTorque - frictionTorque;

        if (config.debugConfig.pivotTorqueDebug()) {
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

}
