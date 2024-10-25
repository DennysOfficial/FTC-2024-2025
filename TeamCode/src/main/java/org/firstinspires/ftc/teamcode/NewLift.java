package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;

public class NewLift extends ControlAxis{

    @Override
    void initMotors() {
        motors.addMotor(config.deviceConfig.rightLift, DcMotorSimple.Direction.FORWARD);
        motors.addMotor(config.deviceConfig.leftLift, DcMotorSimple.Direction.REVERSE);
    }

    @Override
    void updatePositionPIDCoefficients() {
       // positionPid.setCoefficients();
    }
    @Override
    void updateVelocityPIDCoefficients() {
       // velocityPid.setCoefficients();
    }

    public NewLift(OpMode opMode, RobotConfig config){
        super(opMode,config, "Lift");
    }


    public void update(double deltaTime) {
        updateEssentials(deltaTime);

    }
}
