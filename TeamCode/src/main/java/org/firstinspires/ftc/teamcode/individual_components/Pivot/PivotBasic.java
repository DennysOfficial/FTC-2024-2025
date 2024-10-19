package org.firstinspires.ftc.teamcode.individual_components.Pivot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.motionControl.MultiMotor;

@Config
public class PivotBasic {

    static final int encoderCountsPerRevMotor = 28;
    static final double finalGearRatio = 1. / 200.; // rotations of final over rotations of motor
    static final double encoderCountsPerRevFinal = encoderCountsPerRevMotor / finalGearRatio;
    static final double encoderCountsPerDeg = encoderCountsPerRevFinal / 360;

    //@Config
    public static class motorProperties {
        public static double maxSpeedRPM = 6000;
        public static double stallAmps;
    }

    static double minAngle = -87; // measured from vertical forward is positive
    static double maxAngle = 90;

    final double maxPIDPower = 1f; // mostly a testing thing to stop the robot from committing scooter ankle
    public static boolean debugModeActive = false;
    LinearOpMode opMode;
    RobotConfig config;

    /**
     * degrees from vertical: forward is positive
     */
    protected double targetAngle = 0;

    protected DcMotor.RunMode runMode = DcMotor.RunMode.RUN_TO_POSITION;

    protected MultiMotor motors;

    public PivotBasic(LinearOpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;

        motors = new MultiMotor(opMode.hardwareMap);

        motors.addMotor(config.deviceConfig.leftPivot, DcMotorSimple.Direction.FORWARD);
        motors.addMotor(config.deviceConfig.rightPivot, DcMotorSimple.Direction.REVERSE);

        motors.setTargetPosition(0);
        motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTorque(double targetTorque) {

        double motorSpeedRPM = motors.getVelocity() / encoderCountsPerRevMotor;

        double battery = config.sensorData.getBatteryVoltage() / 12.0;

        motors.setPower((targetTorque + motorSpeedRPM / PivotAdvanced.motorProperties.maxSpeedRPM) / battery);

        if (config.debugConfig.pivotTorqueDebug()) {
            opMode.telemetry.addData("targetTorque", targetTorque);
            opMode.telemetry.addData("motorCurrent", motors.getCurrent(CurrentUnit.AMPS));
            //opMode.telemetry.addData("motorSpeed", motorSpeedRPM);
            opMode.telemetry.addData("motorPower", motors.getPower());
        }
    }

    /**
     * @return velocity in degrees per second
     */
    public double getVelocityDPS() {
        return motors.getVelocity() / encoderCountsPerDeg;
    }


    public void directControlBasic() {
        motors.setPower(config.inputMap.getPivotStick() * config.sensitivities.getPivotRate());
    }

    /**
     * gets the current angle of the pivot
     *
     * @return degrees from the vertical: forward is positive
     */
    public double getAngle() {
        return motors.getCurrentPosition() / encoderCountsPerDeg;
    }

    public double getTargetAngle(){
        return targetAngle;
    }



}
