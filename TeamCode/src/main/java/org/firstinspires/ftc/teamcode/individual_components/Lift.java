package org.firstinspires.ftc.teamcode.individual_components;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;
import org.firstinspires.ftc.teamcode.motionControl.MultiMotor;

public class Lift {

    public final double minRangeInch = 0.1;
    public final double maxRangeInch = 27.3;
    final double encoderCountsPerInch = 4300.0 / 27.0;
    final double maxPower = 1;


    LinearOpMode opMode;
    RobotConfig config;

    protected final MultiMotor motors;


    public Lift(LinearOpMode opMode, RobotConfig config) {
        this.opMode = opMode;
        this.config = config;

        motors = new MultiMotor(opMode.hardwareMap);

        motors.addMotor(config.deviceConfig.rightLift, DcMotorSimple.Direction.FORWARD);
        motors.addMotor(config.deviceConfig.leftLift, DcMotorSimple.Direction.REVERSE);

        motors.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motors.getMotor(config.deviceConfig.leftLift).setMotorDisable();
    }

    /**
     * moves the lift based on the sick input and sensitivity defined by the config
     *
     * @param deltaTime the change in time (seconds) since the method was last called
     */
    public void directControl(double deltaTime) {
        setTargetPositionInch(motors.getTargetPosition() + opMode.gamepad2.left_stick_y * config.getLiftRate() * deltaTime);
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
