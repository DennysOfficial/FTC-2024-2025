package org.firstinspires.ftc.teamcode.individual_components.Pivot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;
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
        public static double extendedGComp = .2;
        public static double retractedGComp = .05;
        public static double maxExtension = 27.3;
    }

    @Config
    public static class pivotPIDCon {
        public static double posKP = 0.05;
        public static double posKI = 0;
        public static double posKD = 0.002;

        public static double velKP = 0;
        public static double velKI = 0;
        public static double velKD = 0;
        public static double velFeedforward = 0;
    }

    public enum PivotControlSate {
        directControl,
        PIDPositionControl,
        PIDVelocityControl,
        testing
    }

    public PivotControlSate controlSate = PivotControlSate.directControl;

    CustomPID pivotPositionPID;
    CustomPID pivotVelocityPID;

    double targetVelocity = 0;


    PositionDerivatives positionDerivatives; // calculates velocity, acceleration, and jerk

    public PivotAdvanced(LinearOpMode opMode, RobotConfig config) {
        super(opMode, config);
        pivotPositionPID = new CustomPID(opMode, config, "PivotPosition");
        pivotVelocityPID = new CustomPID(opMode, config, "pivotVelocity");

        positionDerivatives = new PositionDerivatives(getAngle());
    }

    public void update(double deltaTime, double liftExtensionInch) {

        positionDerivatives.update(getAngle(), deltaTime);

        if (config.inputMap.getAbort())
            controlSate = PivotControlSate.directControl;

        //if (config.inputMap.getPivotStick() > config.getAutoAbortThreshold() || config.inputMap.getPivotStick() < -config.getAutoAbortThreshold())
        //controlSate = PivotControlSate.directControl;

        opMode.telemetry.addData("pivotStick", config.inputMap.getPivotStick());
        opMode.telemetry.addData("pivotStatus", controlSate);


        switch (controlSate) {
            case directControl:
                directControlFancy(liftExtensionInch);
                break;
            case PIDPositionControl:
                pivotPositionPID.setCoefficients(pivotPIDCon.posKP, pivotPIDCon.posKI, pivotPIDCon.posKD);
                setTorque(calculateNetTorque(pivotPositionPID.runPID(targetAngle, getAngle(), deltaTime, positionDerivatives.getVelocity()), liftExtensionInch));
                break;
            case PIDVelocityControl:
                pivotVelocityPID.setCoefficients(pivotPIDCon.velKP, pivotPIDCon.velKI, pivotPIDCon.velKD);
                setTorque(pivotPIDCon.velFeedforward * targetVelocity + calculateNetTorque(pivotVelocityPID.runPID(targetVelocity, positionDerivatives.getVelocity(), deltaTime, positionDerivatives.getAcceleration()), liftExtensionInch));
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
        setTorque(calculateNetTorque(config.inputMap.getPivotStick() * config.sensitivities.getPivotSensitivity(), liftExtensionInch) - dampingTorque);
        if (config.debugConfig.pivotTorqueDebug())
            opMode.telemetry.addData("dampingTorque", dampingTorque);
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
        controlSate = PivotControlSate.PIDPositionControl;
    }

    public void setTargetVelocity(double targetVelocityDPS) {
        this.targetVelocity = targetVelocityDPS;
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

    public void setTorque(double torque) {
        motors.setTorque(torque, positionDerivatives.getVelocity() * encoderCountsPerDeg);
    }

}
