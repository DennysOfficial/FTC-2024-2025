package org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.Controllable;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;

import kotlin.jvm.functions.Function0;

@Config
public class FieldCentricDrive extends DriveModeBase {

    IMU imu;

    MotorEx frontLeft = new MotorEx(frontLeftDrive);
    MotorEx frontRight = new MotorEx(frontRightDrive);
    MotorEx backLeft = new MotorEx(backLeftDrive);
    MotorEx backRight = new MotorEx(backRightDrive);

    Controllable[] motors = new Controllable[]{frontLeft, frontRight, backLeft, backRight};

    Function0<Float> forwardBackward = () -> (float) (config.playerOne.forwardAxis.getValue() * config.sensitivities.getForwardSensitivity() * getSensitivityMod());
    Function0<Float> strafe = () -> (float) (config.playerOne.strafeAxis.getValue() * config.sensitivities.getStrafingSensitivity() * getSensitivityMod());
    Function0<Float> yaw = () -> (float) (config.playerOne.turnAxis.getValue() * config.sensitivities.getTurningSensitivity() * getSensitivityMod());

    MecanumDriverControlled vroom = new MecanumDriverControlled(motors, forwardBackward, strafe, yaw, false, imu);


    public FieldCentricDrive(OpMode opMode, RobotConfig config) {
        super(opMode, config);
        imu = opMode.hardwareMap.get(IMU.class, "imu");
    }


    public float getSensitivityMod() {
        float SensitivityModifier = config.sensitivities.getDriveSensitivity();
        if (config.playerOne.slowDown.getState()){SensitivityModifier = config.sensitivities.getSlowDownModifier();}
        return SensitivityModifier;
    }


    @Override
    public void updateDrive(double deltaTime) {
        vroom.update();
    }
}
