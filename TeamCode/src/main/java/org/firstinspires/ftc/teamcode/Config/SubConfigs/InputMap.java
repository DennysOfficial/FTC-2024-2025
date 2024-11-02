package org.firstinspires.ftc.teamcode.Config.SubConfigs;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class InputMap {

    Gamepad gamepad1;
    Gamepad gamepad2;

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

    public double getLiftStick() {
        return -1 * gamepad2.left_stick_y;
    }

    public double getPivotStick() {
        return -1 * gamepad2.right_stick_y;
    }

    public boolean getPinchButton() {
        return gamepad2.right_bumper;
    }

    public boolean getIntakeButton() {
        return gamepad2.right_bumper;
    }

    public boolean getOuttakeButton() {
        return gamepad2.left_bumper;
    }

    public boolean getAbort() {
        return gamepad1.dpad_left ||  gamepad2.dpad_left;
    }

    public boolean getUnAbort(){return gamepad1.dpad_right ||  gamepad2.dpad_right;}

    public boolean getFlapButton() {
        return gamepad2.left_trigger > 0.5;
    }

    public boolean getSlowDown(){return gamepad1.left_trigger > 0.5;}

    public boolean getSpeedUp(){return gamepad1.right_trigger > 0.5;}

    ElapsedTime doubleClickTimer = new ElapsedTime();


}
