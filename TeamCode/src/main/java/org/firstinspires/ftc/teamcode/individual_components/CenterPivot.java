package org.firstinspires.ftc.teamcode.individual_components;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;
@Config
public class CenterPivot{

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

    ColorRangeSensor colorSens;

    public enum HomingStatus{
        notHomed,
        roughHoming,
        fineHoming,
        homed
    }
    private HomingStatus homingStatus = HomingStatus.notHomed;

    final int red = 650;
    final int grey = 510;
    final int threshold = (red+grey)/2;



    public static double roughHomingSpeed = 30;
    boolean lastHomingSensorReading;

    public boolean homingSensorTriggered(){
        opMode.telemetry.addData("red",(colorSens.red() > threshold));
        return (colorSens.red() > threshold);
    }

    public void initHoming(){
        homingStatus = HomingStatus.roughHoming;
        lastHomingSensorReading = homingSensorTriggered();
    }


    public void homingRoutine(){
        switch (homingStatus){
            case homed:
                return;
            case roughHoming:
                if(homingSensorTriggered())
                    setVelocity(roughHomingSpeed);
                else
                    setVelocity(-roughHomingSpeed);
                //if(lastHomingSensorReading != homingSensorTriggered()){
                  //  homingStatus = HomingStatus.fineHoming;
                //}
        }
    }

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

        colorSens = opMode.hardwareMap.get(ColorRangeSensor.class,"ColorSense");

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
        //if (homingStatus != HomingStatus.homed)
         //   return;

        targetAngle = MathUtils.clamp(targetAngle,minAngle,maxAngle);

        setPosition((int)(targetAngle*encoderCountsPerDeg));

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

    public void setPosition(int targetPosition){
        pivotMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pivotMotorR.setPower(maxPIDPower);
        pivotMotorL.setPower(maxPIDPower);

        pivotMotorR.setTargetPosition(targetPosition);
        pivotMotorL.setTargetPosition(targetPosition);
    }

    public void setVelocity(double targetVelocity){
        pivotMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivotMotorR.setVelocity(-targetVelocity, AngleUnit.DEGREES);
        pivotMotorL.setVelocity(-targetVelocity, AngleUnit.DEGREES);
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
