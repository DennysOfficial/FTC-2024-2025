package org.firstinspires.ftc.teamcode.individual_components;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.motionControl.CustomPID;
import org.firstinspires.ftc.teamcode.motionControl.MultiMotor;
import org.firstinspires.ftc.teamcode.motionControl.PositionDerivatives;

@Config
public class Lift {

    public final double minRangeInch = 0.1;
    public final double maxRangeInch = 27.3;
    final double encoderCountsPerInch = 4300.0 / 27.0;
    final double maxPower = 1;

    public static double gCompMultiplier = 0.069;


    LinearOpMode opMode;
    RobotConfig config;

    public enum LiftControlSate {
        directControl,
        PIDControl,
        testing
    }

    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public LiftControlSate controlSate = LiftControlSate.directControl;
    protected final MultiMotor motors;

    PositionDerivatives positionDerivatives;
    CustomPID liftPid;
    public double targetPosition = 0.1;


    public Lift(LinearOpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;

        motors = new MultiMotor(opMode.hardwareMap);

        motors.addMotor(config.deviceConfig.rightLift, DcMotorSimple.Direction.FORWARD);
        motors.addMotor(config.deviceConfig.leftLift, DcMotorSimple.Direction.REVERSE);

        motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motors.getMotor(config.deviceConfig.leftLift).setMotorDisable();

        positionDerivatives = new PositionDerivatives(getPositionInch());


        liftPid = new CustomPID(opMode, config, "Lift");
    }

    public void update(double deltaTime, double pivotAngle) {



        positionDerivatives.update(getPositionInch(), deltaTime);

        if (config.inputMap.getAbort())
            controlSate = LiftControlSate.directControl;

        if (config.inputMap.getLiftStick() > config.getAutoAbortThreshold() || config.inputMap.getLiftStick() < -config.getAutoAbortThreshold())
            controlSate = LiftControlSate.directControl;

        switch (controlSate) {
            case directControl:
                motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                fancyDirectControl(pivotAngle);
                break;

            case PIDControl:
                liftPid.setCoefficients(Kp, Ki, Kd);
                setCompensatedForce(liftPid.runPID(targetPosition, getPositionInch(), deltaTime, positionDerivatives.getVelocity()), pivotAngle);
                break;

            case testing:

                break;
        }

        if (config.debugConfig.liftPositionAndDerivativesDebug()) {
            opMode.telemetry.addData("liftPosition", getPositionInch());
            opMode.telemetry.addData("liftVelocity", positionDerivatives.getVelocity());
            opMode.telemetry.addData("liftAcceleration", positionDerivatives.getAcceleration());
        }
    }

    public void fancyDirectControl(double pivotAngleDeg) {
        double targetForce = config.inputMap.getLiftStick() * config.sensitivities.getLiftSensitivity();
        setCompensatedForce(targetForce, pivotAngleDeg);
    }

    public void setCompensatedForce(double targetForce, double pivotAngleDeg) {
        double outputForce = targetForce - calcGravityForce(pivotAngleDeg);
        setTorque(outputForce);
    }

    public double calcGravityForce(double pivotAngleDeg) {
        return -Math.cos(Math.toRadians(pivotAngleDeg)) * gCompMultiplier;
    }

    public void setTorque(double targetTorque) {//TODO
        motors.setPower(targetTorque);
    }

    /**
     * moves the lift based on the sick input and sensitivity defined by the config
     *
     * @param deltaTime the change in time (seconds) since the method was last called
     */
    public void directControl(double deltaTime) {
        setTargetPositionInch(motors.getTargetPosition() + opMode.gamepad2.left_stick_y * config.sensitivities.getLiftRate() * deltaTime);
    }

    public void directControlBasic() {
        motors.setPower(opMode.gamepad2.left_stick_y);
    }

    /**
     * gets the current extension of the lift
     *
     * @return inches from the bottom of the lift's travel
     */
    public double getPositionInch() {
        return motors.getCurrentPosition() / encoderCountsPerInch;
    }

    /**
     * sets the target extension of the lift
     *
     * @param positionInch inches from the bottom of the lift's travel
     */
    public void setTargetPositionInch(double positionInch) {
        motors.setTargetPosition((int) (MathUtils.clamp(positionInch, minRangeInch, maxRangeInch) * encoderCountsPerInch));
    }

    /**
     * gets the current targeted extension of the lift
     *
     * @return inches from the bottom of the lift's travel
     */
    public double getTargetPositionInch() {
        return motors.getTargetPosition();
    }


}
