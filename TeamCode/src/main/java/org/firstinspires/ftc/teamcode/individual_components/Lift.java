package org.firstinspires.ftc.teamcode.individual_components;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;
import org.firstinspires.ftc.teamcode.misc.PID_Stuff.CustomPID;

public class Lift {

    public final double MinRangeInch = 0.1;
    public final double MaxRangeInch = 420;
    final double encoderCountsPerInch = 4300.0 / 27.0;
    final double maxPower = 1;

    public boolean debugModeActive = false;
    LinearOpMode opMode;
    RobotConfig config;
    private double targetPosition = 0;
    private final DcMotorEx liftMotor1;
    private final DcMotorEx liftMotor2;

    public Lift(LinearOpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;

        liftMotor1 = opMode.hardwareMap.get(DcMotorEx.class, "1-Lift");
        liftMotor2 = opMode.hardwareMap.get(DcMotorEx.class, "2-Lift");

        liftMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        //liftMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);

        liftMotor1.setMotorEnable();
        liftMotor2.setMotorEnable();

        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //this resets the encoder
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor2.setTargetPosition(0);
        liftMotor1.setTargetPosition(0);

        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //this means the lift motors use encoders
        liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        targetPosition = MathUtils.clamp(targetPosition, MinRangeInch, MaxRangeInch);

        liftMotor2.setTargetPosition((int) (targetPosition * encoderCountsPerInch));
        liftMotor1.setTargetPosition((int) (targetPosition * encoderCountsPerInch));

        if (debugModeActive) {
            opMode.telemetry.addLine();
            opMode.telemetry.addData("Lift: ", "target position %4.2f, actual position %4.2f", targetPosition, getPositionInch());
        }
    }

    public void directControlNoPID() {

        double targetPower = opMode.gamepad2.left_stick_y;

        liftMotor2.setPower(targetPower);
        liftMotor1.setPower(targetPower);
    }

    /**
     * runs {@link CustomPID} in order to calculate the power that should be sent to the motors.
     * <p>
     * THIS NEEDS TO BE RUN CONTINUOUSLY or the lift with probably commit scooter ankle
     *
     * @param deltaTime the change in time (seconds) since the method was last called
     */
    public void runLiftPID(double deltaTime) {


    }

    /**
     * gets the current extension of the lift
     *
     * @return encoder counts from the bottom of the lift's travel
     */
    public double getRawPosition() {
        return (liftMotor2.getCurrentPosition() + liftMotor1.getCurrentPosition()) * 0.5;
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
