package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings;

public class ButtonEdgeDetector {
    boolean previousButtonState;

    public ButtonEdgeDetector() {
        previousButtonState = false;
    }
    public ButtonEdgeDetector(boolean initialButtonState) {
        previousButtonState = initialButtonState;
    }

    public boolean getButtonDown(boolean buttonState) {
        return !previousButtonState && (previousButtonState = buttonState);
    }

    public boolean getButtonUp(boolean buttonState) {
        return previousButtonState && !(previousButtonState = buttonState);
    }
}
