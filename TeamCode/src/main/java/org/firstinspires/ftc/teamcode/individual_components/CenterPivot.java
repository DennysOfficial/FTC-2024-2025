package org.firstinspires.ftc.teamcode.individual_components;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;

public class CenterPivot {

    static final int encoderCountsPerRevMotor = 28;
    static final double finalGearRatio = 1./200.; // rotations of final over rotations of motor
    static final double encoderCountsPerRevFinal = encoderCountsPerRevMotor/finalGearRatio;
    static final double encoderCountsPerDeg = encoderCountsPerRevFinal/360;

    static double minAngle = -90; // measured from vertical forward is positive
    static double maxAngle = 100;

    final double maxPIDPower = 1f; // mostly a testing thing to stop the robot from committing scooter ankle
    public boolean debugModeActive = false;
    LinearOpMode opMode;
    RobotConfig config;

    /**
     * degrees from vertical: forward is positive
     */
    private double targetAngle = 0;
    private DcMotorEx pivotMotorL = null;
    private DcMotorEx pivotMotorR = null;

    public CenterPivot(LinearOpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;


        pivotMotorL = opMode.hardwareMap.get(DcMotorEx.class, config.deviceConfig.leftPivot);
        pivotMotorR = opMode.hardwareMap.get(DcMotorEx.class, config.deviceConfig.rightPivot);

        pivotMotorL.setDirection(DcMotorEx.Direction.REVERSE); //this makes the motor run in reverse
        //liftMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);

        pivotMotorL.setMotorEnable();
        pivotMotorR.setMotorEnable();

        pivotMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //this resets the encoder
        pivotMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pivotMotorR.setTargetPosition(0);
        pivotMotorL.setTargetPosition(0);

        pivotMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pivotMotorR.setPower(maxPIDPower);
        pivotMotorL.setPower(maxPIDPower);

    }

    /**
     * moves the pivot based on stick input
     *
     * @param deltaTime the change in time (seconds) since the method was last called
     */
    public void directControl(double deltaTime) {

        targetAngle += config.getPivotStick() * config.getPivotRate() * deltaTime;
        updatePosition();

    }

    public void updatePosition(){
        targetAngle = MathUtils.clamp(targetAngle,minAngle,maxAngle);

        pivotMotorR.setTargetPosition((int)(targetAngle*encoderCountsPerDeg));
        pivotMotorL.setTargetPosition((int)(targetAngle*encoderCountsPerDeg));

        if (debugModeActive){
            opMode.telemetry.addData("target Angle = ",targetAngle);
            opMode.telemetry.addData("actual Angle = ",getAngle());
            opMode.telemetry.addData("error = ",getAngle() - targetAngle);
        }
    }

    /**
     * sets to motor powers equal to the stick position
     */
    public void directControlNoPID() {

        double targetPower = config.getPivotStick();

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
     * gets the current angle of the pivot
     *
     * @return degrees from the vertical: forward is positive
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
        updatePosition();
    }

    /**
     * gets the current targeted Angle of the lift
     *
     * @return degrees from the vertical: forward is positive
     */
    public double getTargetAngle(){
        return targetAngle;
    }



}
