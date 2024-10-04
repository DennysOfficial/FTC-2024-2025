package org.firstinspires.ftc.teamcode.individual_components.Pivot;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;
@Config
public class PivotBasic {

    static final int encoderCountsPerRevMotor = 28;
    static final double finalGearRatio = 1./200.; // rotations of final over rotations of motor
    static final double encoderCountsPerRevFinal = encoderCountsPerRevMotor/finalGearRatio;
    static final double encoderCountsPerDeg = encoderCountsPerRevFinal/360;

    static double minAngle = -87; // measured from vertical forward is positive
    static double maxAngle = 90;

    final double maxPIDPower = 1f; // mostly a testing thing to stop the robot from committing scooter ankle
    public static   boolean debugModeActive = false;
    LinearOpMode opMode;
    RobotConfig config;

    /**
     * degrees from vertical: forward is positive
     */
    private double targetAngle = 0;
    private DcMotorEx pivotMotorL = null;
    private DcMotorEx pivotMotorR = null;


    public PivotBasic(LinearOpMode opMode, RobotConfig config) {
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
    public void directControlStockPID(double deltaTime) {

        targetAngle += config.getPivotStick() * config.getPivotRate() * deltaTime;
        updatePosition();

    }

    void updatePosition(){
        targetAngle = MathUtils.clamp(targetAngle,minAngle,maxAngle);

        setTargetPosition((int)(targetAngle*encoderCountsPerDeg));

        if (debugModeActive){
            opMode.telemetry.addData("PivotTarget",targetAngle);
            opMode.telemetry.addData("PivotActual",getAngle());
            opMode.telemetry.addData("PivotError = ",getAngle() - targetAngle);
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

        setPower(targetPower);
    }

    public void setPower(double targetPower){
        pivotMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pivotMotorR.setPower(targetPower);
        pivotMotorL.setPower(targetPower);
    }

    public void setTargetPosition(int targetPosition){
        pivotMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pivotMotorR.setPower(maxPIDPower);
        pivotMotorL.setPower(maxPIDPower);

        pivotMotorR.setTargetPosition(targetPosition);
        pivotMotorL.setTargetPosition(targetPosition);
    }

    public void setTargetVelocity(double targetVelocity){
        pivotMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivotMotorR.setVelocity(-targetVelocity, AngleUnit.DEGREES);
        pivotMotorL.setVelocity(-targetVelocity, AngleUnit.DEGREES);
    }

    /**
     * @return velocity in ticks per second
     */
    public double getVelocityTPS(){
        if (pivotMotorL.isMotorEnabled() && pivotMotorR.isMotorEnabled())
            return (pivotMotorR.getVelocity() + pivotMotorL.getVelocity())/2f;

        if (pivotMotorL.isMotorEnabled())
            return pivotMotorL.getVelocity();

        if (pivotMotorR.isMotorEnabled())
            return pivotMotorR.getVelocity();
        return 0;
    }

    /**
     * @return velocity in degrees per second
     */
    public double getVelocityDPS(){
       return getVelocityTPS()/encoderCountsPerDeg;
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
        if (pivotMotorL.isMotorEnabled() && pivotMotorR.isMotorEnabled())
            return (pivotMotorR.getCurrentPosition() + pivotMotorL.getCurrentPosition())/2f;

        if (pivotMotorL.isMotorEnabled())
            return pivotMotorL.getCurrentPosition();

        if (pivotMotorR.isMotorEnabled())
            return pivotMotorR.getCurrentPosition();
        return 0;
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
    public void setTargetAngle(double angle) {
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
