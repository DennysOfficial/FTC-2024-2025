package org.firstinspires.ftc.teamcode.RobotStuff.Config.SubConfigs;

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
        return gamepad2.right_bumper || (gamepad2.right_trigger > 0.2);
    }

    public boolean getOuttakeButton() {
        return gamepad2.left_bumper;
    }

    public boolean getAbort() {return gamepad1.dpad_left;}

    public boolean getUnAbort() {return gamepad1.dpad_right;}

    public boolean getElbowLeft() {return gamepad1.x;}

    public boolean getElbowRight() {return gamepad1.b;}

    public boolean getFlapButton() {
        return gamepad2.left_trigger > 0.5;
    }

    public boolean getSlowDown() {
        return gamepad1.left_trigger > 0.5;
    }

    public boolean getAccurateDrive() {return gamepad1.right_trigger > 0.5;} // don't bind gamepad1.right_trigger to anything because i am using it

    ElapsedTime doubleClickTimer = new ElapsedTime();


}
