package org.firstinspires.ftc.teamcode.individual_components.Pivot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.motionControl.MultiMotor;
import org.firstinspires.ftc.teamcode.motionControl.MultiTorqueMotor;

public class PivotBasic {

    static final int encoderCountsPerRevMotor = 28;
    static final double finalGearRatio = 1. / 200.; // rotations of final over rotations of motor
    static final double encoderCountsPerRevFinal = encoderCountsPerRevMotor / finalGearRatio;
    static final double encoderCountsPerDeg = encoderCountsPerRevFinal / 360;


    static double minAngle = -87; // measured from vertical forward is positive
    static double maxAngle = 90;


    LinearOpMode opMode;
    RobotConfig config;

    /**
     * degrees from vertical: forward is positive
     */
    protected double targetAngle = 0;
    protected double angleOffset = 0;

    protected DcMotor.RunMode runMode = DcMotor.RunMode.RUN_TO_POSITION;

    protected MultiTorqueMotor motors;


    public PivotBasic(LinearOpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;

        motors = new MultiTorqueMotor(opMode.hardwareMap, config.sensorData);

        motors.addMotor(config.deviceConfig.leftPivot, DcMotorSimple.Direction.FORWARD);
        motors.addMotor(config.deviceConfig.rightPivot, DcMotorSimple.Direction.REVERSE);

        motors.setTargetPosition(0);
        motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public double getTargetAngle() {
        return targetAngle;
    }


}
