package org.firstinspires.ftc.teamcode.RobotStuff.Config.SubConfigs;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class InputMap {

    public final Gamepad gamepad1;
    public final Gamepad gamepad2;

    public InputMap(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public double getForwardStick() {
        return gamepad1.left_stick_y;
    } // button mapping

    public double getStrafeStick() {
        return -1 * gamepad1.left_stick_x;
    }

    public double getTurnStick() {
        return gamepad1.right_stick_x;
    }

    public double getRightLiftStick() {
        return -1 * gamepad2.left_stick_y;
    }

    public double getLeftLiftStick() {
        return 0;
    }

    public double getRightPivotStick() {
        return -1 * gamepad2.right_stick_y;
    }

    public double getLeftPivotStick() {
        return 0;
    }

    public boolean getClawOpen() {
        return gamepad1.left_trigger > 0.2;
    }

    public boolean getClawTwistToggleButton() {
        return gamepad1.left_bumper;
    }

    public boolean getIntakeButton() {
        return gamepad2.right_bumper || (gamepad2.right_trigger > 0.2);
    }

    public boolean getOuttakeButton() {
        return gamepad2.left_bumper;
    }

    public boolean getAbort() {
        return false;//gamepad1.dpad_left || gamepad2.dpad_left;
    }

    public boolean getUnAbort() {
        return false;//gamepad1.dpad_right || gamepad2.dpad_right;
    }

    public boolean getFlapButton() {
        return gamepad2.left_trigger > 0.5;
    }

    public boolean getSlowDown() {
        return gamepad1.left_trigger > 0.5;
    }

    public boolean getBrake() {
        return false;
    }

    public boolean getIntakeWristDown() {
        return gamepad2.right_trigger > 0.5;
    }

    public boolean getIntakeForward() {
        return gamepad2.y;
    }
    public boolean getObservationDepositPreset() {
        return gamepad2.b;
    }

    public boolean getYoinkButton() {
        return gamepad1.right_trigger > 0.2;
    }

    public boolean getDriveModeCycleRightButton() {
        return gamepad1.dpad_right;
    }
    public boolean getDriveModeCycleLeftButton() {
        return gamepad1.dpad_left;
    }


    ElapsedTime doubleClickTimer = new ElapsedTime();


}
