package org.firstinspires.ftc.teamcode.individual_components;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;

public class Lift {

    public final double MinRangeInch = 0.1;
    public final double MaxRangeInch = 420;
    final double encoderCountsPerInch = 4300.0 / 27.0;
    final double maxPower = 1;

    public boolean debugModeActive = false;
    LinearOpMode opMode;
    RobotConfig config;
    private double targetPosition = 0;
    private final DcMotorEx liftMotorR;
    private final DcMotorEx liftMotorL;

    public Lift(LinearOpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;

        liftMotorR = opMode.hardwareMap.get(DcMotorEx.class, config.deviceConfig.rightLift);
        liftMotorL = opMode.hardwareMap.get(DcMotorEx.class, config.deviceConfig.leftLift);

        liftMotorR.setDirection(DcMotorEx.Direction.REVERSE);
        //liftMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);

        liftMotorR.setMotorEnable();
        liftMotorL.setMotorEnable();

        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //this resets the encoder
        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotorL.setTargetPosition(0);
        liftMotorR.setTargetPosition(0);

        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * moves the lift based on the sick input and sensitivity defined by the config
     *
     * @param deltaTime the change in time (seconds) since the method was last called
     */
    public void directControl(double deltaTime) {
        targetPosition -= opMode.gamepad2.left_stick_y * config.getLiftRate() * deltaTime;

        updatePosition();
    }

    public void updatePosition() {
        liftMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //this means the lift motors use encoders
        liftMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        targetPosition = MathUtils.clamp(targetPosition, MinRangeInch, MaxRangeInch);

        liftMotorL.setTargetPosition((int) (targetPosition * encoderCountsPerInch));
        liftMotorR.setTargetPosition((int) (targetPosition * encoderCountsPerInch));

        if (debugModeActive) {
            opMode.telemetry.addLine();
            opMode.telemetry.addData("Lift: ", "target position %4.2f, actual position %4.2f", targetPosition, getPositionInch());
        }
    }

    public void directControlNoPID() {

        double targetPower = opMode.gamepad2.left_stick_y;

        liftMotorL.setPower(targetPower);
        liftMotorR.setPower(targetPower);
    }


    /**
     * gets the current extension of the lift
     *
     * @return encoder counts from the bottom of the lift's travel
     */
    public double getRawPosition() {
        return (liftMotorL.getCurrentPosition() + liftMotorR.getCurrentPosition()) * 0.5;
    }

    /**
     * gets the current extension of the lift
     *
     * @return inches from the bottom of the lift's travel
     */
    public double getPositionInch() {
        return getRawPosition() / encoderCountsPerInch;
    }

    /**
     * sets the target extension of the lift
     *
     * @param positionInch inches from the bottom of the lift's travel
     */
    public void setPositionInch(double positionInch) {
        targetPosition = positionInch;
        updatePosition();
    }

    /**
     * gets the current targeted extension of the lift
     *
     * @return inches from the bottom of the lift's travel
     */
    public double getTargetPositionInch() {
        return targetPosition;
    }


}
