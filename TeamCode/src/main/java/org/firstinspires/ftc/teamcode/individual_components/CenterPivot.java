package org.firstinspires.ftc.teamcode.individual_components;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.profiles_and_base_settings.Settings;

public class CenterPivot {

    static final int encoderCountsPerRevMotor = 28;
    static final double finalGearRatio = 1./200.; // rotations of final over rotations of motor
    static final double encoderCountsPerRevFinal = encoderCountsPerRevMotor/finalGearRatio;
    static final double encoderCountsPerDeg = encoderCountsPerRevFinal/360;

    static double minAngle = -90; // measured from vertical forward is positive
    static double maxAngle = 100;

    static float maxRate = 20; // degrees per second

    final double maxPIDPower = 0.5f;
    public boolean debugModeActive = false;
    LinearOpMode opMode;
    Settings settings;
    private double targetAngle = 0;
    private DcMotorEx pivotMotorL = null;
    private DcMotorEx pivotMotorR = null;

    public CenterPivot(LinearOpMode opMode, Settings settings) {
        this.opMode = opMode;
        this.settings = settings;

        pivotMotorL = opMode.hardwareMap.get(DcMotorEx.class, "PivotL");
        pivotMotorR = opMode.hardwareMap.get(DcMotorEx.class, "PivotR");

        pivotMotorL.setDirection(DcMotorEx.Direction.REVERSE);
        //liftMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);

        pivotMotorL.setMotorEnable();
        pivotMotorR.setMotorEnable();

        pivotMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pivotMotorR.setTargetPosition(0);
        pivotMotorL.setTargetPosition(0);

        pivotMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     * moves the lift based on the sick input and sensitivity defined by {@link Settings#getLiftStick()} and {@link Settings#liftSensitivity}
     *
     * @param deltaTime the change in time (seconds) since the method was last called
     */
    public void directControl(double deltaTime) {

        pivotMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pivotMotorR.setPower(maxPIDPower);
        pivotMotorL.setPower(maxPIDPower);

        targetAngle += opMode.gamepad2.left_stick_x * maxRate * deltaTime;

        targetAngle = MathUtils.clamp(targetAngle,minAngle,maxAngle);

        pivotMotorR.setTargetPosition((int)(targetAngle*encoderCountsPerDeg));
        pivotMotorL.setTargetPosition((int)(targetAngle*encoderCountsPerDeg));

        if (debugModeActive){
            opMode.telemetry.addData("target Angle = ",targetAngle);
            opMode.telemetry.addData("actual Angle = ",getAngle());
        }
    }

    /**
     * sets to motor powers equal to the stick
     */
    public void directControlNoPID() {

        double targetPower = opMode.gamepad2.left_stick_x;

        if (debugModeActive){
            opMode.telemetry.addData("motor power = ",targetPower);
        }

        pivotMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pivotMotorR.setPower(targetPower);
        pivotMotorL.setPower(targetPower);
    }
    public void directControlNoPID(double delaTime) {
        directControlNoPID();
    }

    /**
     * gets the current extension of the lift
     *
     * @return encoder counts from the bottom of the lift's travel
     */
    public double getRawEncoder() {
        return (pivotMotorR.getCurrentPosition() + pivotMotorL.getCurrentPosition())/2f;
    }

    /**
     * gets the current extension of the lift
     *
     * @return inches from the bottom of the lift's travel
     */
    public double getAngle() {
        return getRawEncoder() / encoderCountsPerDeg;
    }

    /**
     * sets the target extension of the lift
     *
     * @param angle angle from vertical: forward is positive
     */
    public void setAngle(double angle) {
        targetAngle = angle;
    }

    /**
     * gets the current targeted extension of the lift
     *
     * @return encoder counts from the bottom of the lift's travel
     */
    public double getTargetAngle(){
        return targetAngle;
    }



}
