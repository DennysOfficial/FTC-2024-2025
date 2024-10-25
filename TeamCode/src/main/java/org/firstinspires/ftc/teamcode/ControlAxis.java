package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.individual_components.Lift;
import org.firstinspires.ftc.teamcode.motionControl.CustomPID;
import org.firstinspires.ftc.teamcode.motionControl.MultiMotor;
import org.firstinspires.ftc.teamcode.motionControl.MultiTorqueMotor;
import org.firstinspires.ftc.teamcode.motionControl.PositionDerivatives;

import java.util.Objects;

public abstract class ControlAxis {


    String axisName;

    OpMode opMode;
    RobotConfig config;

    MultiTorqueMotor motors;

    abstract void initMotors();


    public enum ControlState {

        disabled,
        directControl,
        positionControl,
        velocityControl,
        testing
    }

    protected ControlState controlState = ControlState.disabled;

    PositionDerivatives positionDerivatives;

    protected CustomPID positionPid;
    protected CustomPID velocityPid;

    void initPid() {
        positionPid = new CustomPID(opMode, config, axisName + " positionPID");
        velocityPid = new CustomPID(opMode, config, axisName + " velocityPID");
    }

    abstract void updatePositionPIDCoefficients();

    abstract void updateVelocityPIDCoefficients();


    public ControlAxis(OpMode opMode, RobotConfig config, String axisName) {
        this.opMode = opMode;
        this.config = config;
        this.axisName = axisName;

        positionDerivatives = new PositionDerivatives(motors.getCurrentPosition());

        initPid();
        initMotors();
        updatePositionPIDCoefficients();
        updateVelocityPIDCoefficients();
    }




    void updateEssentials(double deltaTime) {
        positionDerivatives.update(motors.getCurrentPosition(), deltaTime);

        if (config.inputMap.getAbort())
            controlState = ControlState.disabled;

        if (controlState == ControlState.disabled) {
            motors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors.setPower(0);
        }
    }


    void updatePositionPID(double targetPosition, double deltaTime, double feedforward) {
        updatePositionPIDCoefficients();
        motors.setTorque(feedforward + positionPid.runPID(targetPosition, motors.getCurrentPosition(), deltaTime, positionDerivatives.getVelocity()), positionDerivatives.getVelocity());
    }

    void updateVelocityPID(double targetVelocity, double deltaTime, double feedforward) {
        updateVelocityPIDCoefficients();
        motors.setTorque(feedforward + velocityPid.runPID(targetVelocity, positionDerivatives.getVelocity(), deltaTime, positionDerivatives.getAcceleration()), positionDerivatives.getVelocity());
    }

}
