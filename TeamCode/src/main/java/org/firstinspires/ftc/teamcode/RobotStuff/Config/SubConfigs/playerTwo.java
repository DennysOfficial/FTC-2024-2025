package org.firstinspires.ftc.teamcode.RobotStuff.Config.SubConfigs;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.rowanmcalpin.nextftc.ftc.gamepad.Button;
import com.rowanmcalpin.nextftc.ftc.gamepad.Joystick;
import com.rowanmcalpin.nextftc.ftc.gamepad.JoystickAxis;
import com.rowanmcalpin.nextftc.ftc.gamepad.Trigger;

import kotlin.jvm.functions.Function0;

public class playerTwo {

    Gamepad gamepad;

    public playerTwo(Gamepad gamepad) {
        if (gamepad == null) {
            throw new NullPointerException("you can't drive without the inputs you goober");
        }
        this.gamepad = gamepad;
    }
    ///////////////////////////////////////--- X/Y/A/B BUTTONS ---/////////////////////////////////////////
    Function0<Boolean> getA = () -> gamepad.a;
    Function0<Boolean> getB = () -> gamepad.b;
    Function0<Boolean> getX = () -> gamepad.x;
    Function0<Boolean> getY = () -> gamepad.y;

    public Button a = new Button(getA);
    public Button b = new Button(getB);
    public Button x = new Button(getX);
    public Button y = new Button(getY);

    public void update_buttons() {
        a.update();
        b.update();
        x.update();
        y.update();
    }


    ///////////////////////////////////////--- BUMPERS ---/////////////////////////////////////////
    Function0<Boolean> getLBumper = () -> gamepad.left_bumper;
    Function0<Boolean> getRBumper = () -> gamepad.right_bumper;

    public Button outtakeBumper = new Button(getLBumper);
    public Button pinchBumper = new Button(getRBumper);

    public void update_bumpers() {
        outtakeBumper.update();
        pinchBumper.update();
    }

    ///////////////////////////////////////--- DPAD ---/////////////////////////////////////////
    Function0<Boolean> getDpadUp = () -> gamepad.dpad_up;
    Function0<Boolean> getDpadDown = () -> gamepad.dpad_down;
    Function0<Boolean> getDpadLeft = () -> gamepad.dpad_left;
    Function0<Boolean> getDpadRight = () -> gamepad.dpad_right;

    public Button dpad_up = new Button(getDpadUp);
    public Button dpad_down = new Button(getDpadDown);
    public Button setLeftArm = new Button(getDpadLeft);
    public Button setRightArm = new Button(getDpadRight);

    public void update_dpad() {
        dpad_up.update();
        dpad_down.update();
        setLeftArm.update();
        setRightArm.update();
    }


    ///////////////////////////////////////--- LEFT JOYSTICK ---/////////////////////////////////////////
    Function0<Float> getLeftX = () -> gamepad.left_stick_x;
    Function0<Float> getLeftY = () -> gamepad.left_stick_y;
    Function0<Boolean> getLeftButton = () -> gamepad.left_stick_button;

    public Joystick left_stick = new Joystick(
            getLeftX,
            getLeftY,
            getLeftButton,
            Sensitivities.playerTwoLeftXAxisThreshold,
            Sensitivities.playerTwoLeftYAxisThreshold,
            Sensitivities.isPlayerTwoLeftYInverted //NextFTC only does vertical inversion for the Joystick object
    );

    public JoystickAxis leftX = new JoystickAxis(
            getLeftX,
            Sensitivities.playerTwoLeftXAxisThreshold,
            Sensitivities.isPlayerTwoLeftXInverted
    );

    public JoystickAxis liftAxis = new JoystickAxis(
            getLeftY,
            Sensitivities.playerTwoLeftYAxisThreshold,
            Sensitivities.isPlayerTwoLeftYInverted
    );

    public Button left_button = new Button(getLeftButton);

    public void update_left() {
        left_stick.update();
        leftX.update();
        liftAxis.update();
        left_button.update();
    }


    ///////////////////////////////////////--- RIGHT JOYSTICK ---/////////////////////////////////////////
    Function0<Float> getRightX = () -> gamepad.right_stick_x;
    Function0<Float> getRightY = () -> gamepad.right_stick_y;
    Function0<Boolean> getRightButton = () -> gamepad.right_stick_button;

    public Joystick right_stick = new Joystick(
            getRightX,
            getRightY,
            getRightButton,
            Sensitivities.playerTwoRightXAxisThreshold,
            Sensitivities.playerTwoRightYAxisThreshold,
            Sensitivities.isPlayerTwoRightYInverted //NextFTC only does vertical inversion for the Joystick object
    );

    public JoystickAxis rightX = new JoystickAxis(
            getRightX,
            Sensitivities.playerTwoRightXAxisThreshold,
            Sensitivities.isPlayerTwoRightXInverted
    );

    public JoystickAxis pivotAxis = new JoystickAxis(
            getRightY,
            Sensitivities.playerTwoRightYAxisThreshold,
            Sensitivities.isPlayerTwoRightYInverted
    );

    public Button right_button = new Button(getRightButton);


    public void update_right() {
        right_stick.update();
        rightX.update();
        pivotAxis.update();
        right_button.update();
    }


    ///////////////////////////////////////--- TRIGGERS ---/////////////////////////////////////////
    Function0<Float> getLTrigger = () -> gamepad.left_trigger;
    Function0<Float> getRTrigger = () -> gamepad.right_trigger;

    public Trigger flapTrigger = new Trigger(getLTrigger, Sensitivities.playerTwoLeftTriggerThreshold);
    public Trigger intakeTrigger = new Trigger(getRTrigger, Sensitivities.playerTwoRightTriggerThreshold);

    public void update_triggers() {
        flapTrigger.update();
        intakeTrigger.update();
    }

    public void update_all() {
        update_buttons();
        update_bumpers();
        update_dpad();
        update_left();
        update_right();
        update_triggers();
    }

}
