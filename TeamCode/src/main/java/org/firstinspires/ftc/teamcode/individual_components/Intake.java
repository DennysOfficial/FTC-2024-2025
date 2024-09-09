package org.firstinspires.ftc.teamcode.individual_components;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.profiles_and_base_settings.Settings;

public class Intake {

    DcMotorEx intakeMotor;
    Servo liftingServo;
    OpMode opMode;
    Settings activeSettings;

    double[] intakeLiftRange = new double[]{0.22, 0.8};

    public Intake(OpMode opMode, Settings activeSettings) {

        this.opMode = opMode;
        this.activeSettings = activeSettings;

        intakeMotor = opMode.hardwareMap.get(DcMotorEx.class, "intake");

        liftingServo = opMode.hardwareMap.get(Servo.class, "intake lift servo");
    }

    double intakePosition = intakeLiftRange[1];
    public void updateIntake(double deltaTime) {
        intakeMotor.setPower(0);

        if (activeSettings.getIntakeInButton()) {
            intakeMotor.setPower(activeSettings.intakeInSpeed);
        }

        if (activeSettings.getIntakeOutButton()) {
            intakeMotor.setPower(activeSettings.intakeOutSpeed);
        }

        if (activeSettings.getIntakeLiftUp()) {
            intakePosition = liftingServo.getPosition() - activeSettings.intakeLiftSensitivity * deltaTime;
        }

        if (activeSettings.getIntakeLiftDown()) {
            intakePosition = liftingServo.getPosition() + activeSettings.intakeLiftSensitivity * deltaTime;
        }

        liftingServo.setPosition(MathUtils.clamp(intakePosition,intakeLiftRange[0],intakeLiftRange[1]));

        opMode.telemetry.addData("lift servo", "position = %f",liftingServo.getPosition());
    }

    public void setIntakeLift(double servoPosition) {
        intakePosition = servoPosition;
    }


}
