package org.firstinspires.ftc.teamcode.individual_components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.misc.PID_Stuff.CustomPID;

public class Lift {

    final double encoderCountsToInch = 4300.0 / 27.0;
    final double MinRange = 50;
    public final double MinRangeInch = MinRange / encoderCountsToInch;
    final double MaxRange = 3185.0;
    public final double MaxRangeInch = MaxRange / encoderCountsToInch;
    final double maxPower = 1;
    final double clampRange = 200; // tries to prevent overshoot if the sensitivity is higher than the max speed the lift is physically capable of
    final boolean liftMotor2HasEncoder = false; // if false it is assumed that the left motor has the encoder

    public boolean debugModeActive = false;
    public double kP = 0.006;
    public double kI = 0.001;
    public double kD = 0.0001;
    LinearOpMode opMode;
    Settings settings;
    private double targetPosition = 0;
    CustomPID PID;
    private DcMotorEx liftMotor1 = null;
    private DcMotorEx liftMotor2 = null;

    public Lift(LinearOpMode opMode, Settings settings) {
        this.opMode = opMode;
        this.settings = settings;

        liftMotor1 = opMode.hardwareMap.get(DcMotorEx.class, "1-Lift");
        liftMotor2 = opMode.hardwareMap.get(DcMotorEx.class, "2-Lift");

        liftMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        //liftMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);

        liftMotor1.setMotorEnable();
        liftMotor2.setMotorEnable();

        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor2.setTargetPosition(0);
        liftMotor1.setTargetPosition(0);

        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        PID = new CustomPID(opMode, settings, kP, kI, kD);
        PID.startPID(getPosition());
        PID.debugMode = debugModeActive;
    }

    /**
     * moves the lift based on the sick input and sensitivity defined by {@link Settings#getLiftStick()} and {@link Settings#liftSensitivity}
     *
     * @param deltaTime the change in time (seconds) since the method was last called
     */
    public void directControl(double deltaTime) {
        targetPosition -= opMode.gamepad2.left_stick_y * settings.liftSensitivity * deltaTime;

        //targetPosition = Math.max(targetPosition, actualPosition - clampRange);
        //targetPosition = Math.min(targetPosition, actualPosition + clampRange); // need work, meant to reduce overshoot but currently limits top speed of the lift and messes with nate's stuff

        runLiftPID(deltaTime);
    }

    public void directControlNoPID() {

        double targetPower = opMode.gamepad2.left_stick_y;

        liftMotor2.setPower(targetPower);
        liftMotor1.setPower(targetPower);
    }

    public void updatePIDF_Coefficient() {
        PID.kP = kP;
        PID.kI = kI;
        PID.kD = kD;
    }
    /**
     * runs {@link CustomPID} in order to calculate the power that should be sent to the motors.
     *
     * THIS NEEDS TO BE RUN CONTINUOUSLY or the lift with probably commit scooter ankle
     *
     * @param deltaTime the change in time (seconds) since the method was last called
     */
    public void runLiftPID(double deltaTime) {

        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetPosition = Math.max(MinRange, targetPosition);
        targetPosition = Math.min(MaxRange, targetPosition);

        double targetPower = PID.runPID(targetPosition, getPosition(), deltaTime);

        liftMotor2.setPower(targetPower);
        liftMotor1.setPower(targetPower);

        if (debugModeActive) {
            opMode.telemetry.addLine();
            opMode.telemetry.addData("Lift: ", "target position %4.2f, actual position %4.2f", targetPosition / encoderCountsToInch, getPositionInch());
        }
    }

    /**
     * gets the current extension of the lift
     *
     * @return encoder counts from the bottom of the lift's travel
     */
    public double getPosition() {
        return (liftMotor2HasEncoder) ? liftMotor2.getCurrentPosition() : liftMotor1.getCurrentPosition();
    }
    /**
     * gets the current extension of the lift
     *
     * @return inches from the bottom of the lift's travel
     */
    public double getPositionInch() {
        return getPosition() / encoderCountsToInch;
    }
    /**
     * sets the target extension of the lift
     *
     * @param position encoder counts from the bottom of the lift's travel
     */
    public void setPosition(double position) {
        targetPosition = position;
    }
    /**
     * sets the target extension of the lift
     *
     * @param positionInch inches from the bottom of the lift's travel
     */
    public void setPositionInch(double positionInch) {
        targetPosition = encoderCountsToInch * positionInch;
    }
    /**
     * gets the current targeted extension of the lift
     *
     * @return encoder counts from the bottom of the lift's travel
     */
    public double getTargetPosition(){
        return targetPosition;
    }
    /**
     * gets the current targeted extension of the lift
     *
     * @return inches from the bottom of the lift's travel
     */
    public double getTargetPositionInch(){
        return targetPosition / encoderCountsToInch;
    }


}
