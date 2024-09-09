package org.firstinspires.ftc.teamcode.profiles_and_base_settings;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Settings {

    LinearOpMode opMode;

    public Settings(LinearOpMode opModeTemp) {
        opMode = opModeTemp;
    }

    public double getForwardStick() {
        return -1 * opMode.gamepad1.left_stick_y;
    } // button mapping

    public double getStrafeStick() {
        return opMode.gamepad1.left_stick_x;
    }

    public double getTurnStick() {
        return opMode.gamepad1.right_stick_x;
    }

    public float driveSensitivity = 1;
    public float forwardSensitivity = 1; // basic driving sensitivities only relative to each other
    public float turningSensitivity = 1;
    public float strafingSensitivity = 1;


    public double getLiftStick() {
        return opMode.gamepad2.left_stick_y;
    }

    public float liftSensitivity = 1000;

    public double getPivotStick() {
        return opMode.gamepad2.left_stick_x;
    }

    public float pivotSensitivity = 0.1f;

    public double getWrist1Stick() {
        return -1 * opMode.gamepad2.right_stick_y;
    }

    public double getWrist2Stick() {
        return opMode.gamepad2.right_stick_x;
    }

    public float wrist1Sensitivity = 50;
    public float wrist2Sensitivity = 50;

    public double getIK_XStick() {
        return opMode.gamepad2.left_stick_x;
    }

    public double getIK_YStick() {
        return opMode.gamepad2.left_stick_y;
    }

    public double getIK_GrabberTiltStick() {
        return opMode.gamepad2.right_stick_y;
    }

    public float IK_SensitivityX = 7;
    public float IK_SensitivityY = -10;
    public float IK_GrabberTiltSensitivity = 100;

    public boolean getIK_ToggleControlRangeButton() {
        return opMode.gamepad2.a;
    }

    public boolean getGrabber1() {
        return opMode.gamepad2.left_bumper;
    }

    public boolean getGrabber2() {
        return opMode.gamepad2.right_bumper;
    }

    public boolean getIntakeLiftUp(){
        return opMode.gamepad1.b;
    }
    public boolean getIntakeLiftDown(){
        return opMode.gamepad1.a;
    }
    public double intakeLiftSensitivity = 0.5;
    public boolean getPosPreFloor() {
        return opMode.gamepad2.dpad_right;
    }

    public boolean getPosPreBoard() {
        return opMode.gamepad2.dpad_left;
    }
    public boolean getButtonMoveToIntake() {
        return opMode.gamepad2.dpad_right;
    }

    public double intakeInSpeed = 0.5;
    public double intakeOutSpeed = -0.8;

    public boolean getIntakeInButton() {
        return opMode.gamepad1.right_bumper;
    }

    public boolean getIntakeOutButton() {
        return opMode.gamepad1.left_bumper;
    }


    public DcMotorEx.Direction leftFrontDriveDir = DcMotorEx.Direction.REVERSE; //motor directions
    public DcMotorEx.Direction leftBackDriveDir = DcMotorEx.Direction.FORWARD;
    public DcMotorEx.Direction rightFrontDriveDir = DcMotorEx.Direction.REVERSE;
    public DcMotorEx.Direction rightBackDriveDir = DcMotorEx.Direction.FORWARD;

    public enum Grabbers{
        intakeGrabber,
        groundGrabber
    }

    public Grabbers activeGrabber = Grabbers.intakeGrabber;

    public double pivotArmLength = 12;

}