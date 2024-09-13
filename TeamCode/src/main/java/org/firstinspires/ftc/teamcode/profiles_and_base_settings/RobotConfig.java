package org.firstinspires.ftc.teamcode.profiles_and_base_settings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class RobotConfig {

    LinearOpMode opMode;

    public RobotConfig(LinearOpMode opModeTemp) {
        opMode = opModeTemp;
    }
    public static final float driveSensitivity = 1;
    public static final float forwardSensitivity = 1; // basic driving sensitivities only relative to each other
    public static final float turningSensitivity = 1;
    public static final float strafingSensitivity = 1;
    public static final float liftSensitivity = 1000;
    public static final float pivotSensitivity = 0.1f;
    public static final float wrist1Sensitivity = 50;
    public static final float wrist2Sensitivity = 50;
    public static final float IK_SensitivityX = 7;
    public static final float IK_SensitivityY = -10;
    public static final float IK_GrabberTiltSensitivity = 100;
    public static final double intakeLiftSensitivity = 0.5;
    public static final double intakeInSpeed = 0.5;
    public static final double intakeOutSpeed = -0.8;
    public static final DcMotorEx.Direction leftFrontDriveDir = DcMotorEx.Direction.REVERSE; //motor directions
    public static final DcMotorEx.Direction leftBackDriveDir = DcMotorEx.Direction.FORWARD;
    public static final DcMotorEx.Direction rightFrontDriveDir = DcMotorEx.Direction.REVERSE;
    public static final DcMotorEx.Direction rightBackDriveDir = DcMotorEx.Direction.FORWARD;
    public static final Settings.Grabbers activeGrabber = Settings.Grabbers.intakeGrabber;
    public static final double pivotArmLength = 12;

    public double getForwardStick() {
        return -1 * opMode.gamepad1.left_stick_y;
    } // button mapping

    public double getStrafeStick() {
        return opMode.gamepad1.left_stick_x;
    }

    public double getTurnStick() {
        return opMode.gamepad1.right_stick_x;
    }

    public double getLiftStick() {
        return opMode.gamepad2.left_stick_y;
    }

    public double getPivotStick() {
        return opMode.gamepad2.left_stick_x;
    }

    public double getWrist1Stick() {
        return -1 * opMode.gamepad2.right_stick_y;
    }

    public double getWrist2Stick() {
        return opMode.gamepad2.right_stick_x;
    }

    public double getIK_XStick() {
        return opMode.gamepad2.left_stick_x;
    }

    public double getIK_YStick() {
        return opMode.gamepad2.left_stick_y;
    }

    public double getIK_GrabberTiltStick() {
        return opMode.gamepad2.right_stick_y;
    }

    public boolean getIK_ToggleControlRangeButton() {
        return opMode.gamepad2.a;
    }

    public boolean getGrabber1() {
        return opMode.gamepad2.left_bumper;
    }

    public boolean getGrabber2() {
        return opMode.gamepad2.right_bumper;
    }

    public boolean getIntakeLiftUp() {
        return opMode.gamepad1.b;
    }

    public boolean getIntakeLiftDown() {
        return opMode.gamepad1.a;
    }

    public boolean getPosPreFloor() {
        return opMode.gamepad2.dpad_right;
    }

    public boolean getPosPreBoard() {
        return opMode.gamepad2.dpad_left;
    }

    public boolean getButtonMoveToIntake() {
        return opMode.gamepad2.dpad_right;
    }

    public boolean getIntakeInButton() {
        return opMode.gamepad1.right_bumper;
    }

    public boolean getIntakeOutButton() {
        return opMode.gamepad1.left_bumper;
    }

    public enum Grabbers {
        intakeGrabber,
        groundGrabber
    }

}
