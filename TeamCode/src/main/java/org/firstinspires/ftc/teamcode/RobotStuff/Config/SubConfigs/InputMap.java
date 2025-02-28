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

    /// drive control ##################################################################################################################################################
    public double getForwardStick() {
        return gamepad1.left_stick_y;
    } // button mapping

    public double getStrafeStick() {
        return -1 * gamepad1.left_stick_x;
    }

    public double getTurnStick() {
        return gamepad1.right_stick_x;
    }


    public boolean getSlowDown() {
        return gamepad1.left_trigger > 0.5;
    }

    public boolean getBrake() {
        return false;
    }

    /// Pivot and lift control ##################################################################################################################################################

    public double getLiftStick() {
        return -1 * gamepad2.left_stick_y;
    }

    public double getPivotStick() {
        return -1 * gamepad2.right_stick_y;
    }

    /// claw control ##################################################################################################################################################

    public boolean getSpecimenClawOpen() {
        return gamepad1.left_trigger > 0.2;
    }

    public double getYoinkTrigger() {
        return gamepad1.right_trigger;
    }
    public boolean getYoinkButton() {
        return getYoinkTrigger() > 0.2;
    }

    /// intake control ##################################################################################################################################################

    public boolean getIntakeButton() {
        return gamepad2.right_bumper || (gamepad2.right_trigger > 0.2);
    }

    public boolean getOuttakeButton() {
        return gamepad2.left_bumper;
    }

    public boolean getFlapButton() {
        return gamepad2.left_trigger > 0.5;
    }

    /// abort/unabort ##################################################################################################################################################

    public boolean getAbort() {
        return false;//gamepad1.dpad_left || gamepad2.dpad_left;
    }

    public boolean getUnAbort() {
        return false;//gamepad1.dpad_right || gamepad2.dpad_right;
    }

    /// specimen Presets ##################################################################################################################################################

    public boolean getSpecimenHangButton() {
        return gamepad2.x;
    }

    public boolean getSpecimenCollectButton() {
        return gamepad2.a;
    }

    public boolean getSpecimenRestButton() {
        return false;
    }

    /// sample Presets ##################################################################################################################################################

    public boolean getIntakeForward() {
        return gamepad2.y;
    }

    public boolean getObservationDepositPreset() {
        return gamepad2.b;
    }

    public boolean getBasketDepositPreset() {
        return gamepad2.dpad_up;
    }

    public boolean getClawOpenButton() {
        return gamepad1.a;
    }
    public boolean getClawCloseButton() {
        return gamepad1.x;
    }

    /// testing / misc ##################################################################################################################################################

    public boolean getDriveModeCycleRightButton() {
        return gamepad1.dpad_right;
    }

    public boolean getDriveModeCycleLeftButton() {
        return gamepad1.dpad_left;
    }

}
