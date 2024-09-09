package org.firstinspires.ftc.teamcode.misc;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.individual_components.Grabber;
import org.firstinspires.ftc.teamcode.individual_components.Lift;
import org.firstinspires.ftc.teamcode.individual_components.Pivot;
import org.firstinspires.ftc.teamcode.individual_components.Wrist;
import org.firstinspires.ftc.teamcode.profiles_and_base_settings.Settings;

import java.util.ArrayList;
import java.util.List;

public class MotionControl {

    public Pose pose;
    Pivot pivot;
    Lift lift;
    Wrist wrist;
    Grabber grabber;
    ElapsedTime runtime;
    OpMode opMode;
    Settings settings;
    boolean pivotActive = false;
    double pivotStartPosition;
    double pivotEndPosition;
    double pivotDuration;
    double pivotStartTime;
    boolean liftActive = false;
    double liftStartPosition;
    double liftEndPosition;
    double liftDuration;
    double liftStartTime;
    double pivotArmLength = 14; // inches

    public MotionControl(Pivot pivot, Lift lift, Wrist wrist, Grabber grabber, ElapsedTime runtime, OpMode opMode, Settings settings) {
        this.pivot = pivot;
        this.lift = lift;
        this.wrist = wrist;
        this.grabber = grabber;
        this.runtime = runtime;
        this.opMode = opMode;
        this.settings = settings;

        pose = new Pose();

        pose.targetGrabberAngle = 30; // straight down maybe
    }

    /**
     * linearly interpolates between point 1 and point 2 by interpolation amount
     *
     * @param interpolationAmount normally ranges from 0-1
     */
    double lerp(double point1, double point2, double interpolationAmount) {
        double pointDelta = point2 - point1;
        return point1 + pointDelta * interpolationAmount;
    }

    /**
     * sinusoidally interpolates between point 1 and point 2 by interpolation amount
     *
     * @param interpolationAmount clamped from 0-1
     */
    double smoothLerp(double point1, double point2, double interpolationAmount) {
        double pointDelta = point2 - point1;
        interpolationAmount = MathUtils.clamp(interpolationAmount, 0, 1);
        return point1 + pointDelta * Math.sin(Math.PI * interpolationAmount / 2.0);
    }

    public void smoothMovePivot(double startPosition, double endPosition, double duration) {
        if (pivotActive)
            return;
        pivotActive = true;
        pivotStartPosition = startPosition;
        pivotEndPosition = endPosition;
        pivotDuration = duration;
        pivotStartTime = runtime.seconds();
    }

    public void smoothMovePivot(double endPosition, double duration) {
        if (pivotActive)
            return;
        pivotActive = true;
        pivotStartPosition = pivot.getTargetPositionDeg();
        pivotEndPosition = endPosition;
        pivotDuration = duration;
        pivotStartTime = runtime.seconds();
    }

    public boolean pivotBusy() {
        return pivotActive;
    }

    public void smoothMoveLift(double startPosition, double endPosition, double duration) {
        if (liftActive)
            return;
        liftActive = true;
        liftStartPosition = startPosition;
        liftEndPosition = endPosition;
        liftDuration = duration;
        liftStartTime = runtime.seconds();
    }

    public void smoothMoveLift(double endPosition, double duration) {
        if (liftActive)
            return;
        liftActive = true;
        liftStartPosition = lift.getPositionInch();
        liftEndPosition = endPosition;
        liftDuration = duration;
        liftStartTime = runtime.seconds();
    }

    public boolean liftBusy() {
        return liftActive;
    }

    public void updateMotionControl() {
        double time = runtime.seconds();

        if (pivotActive) {
            if (time - pivotStartTime > pivotDuration)
                pivotActive = false;

            pivot.setTargetPositionDeg(smoothLerp(pivotStartPosition, pivotEndPosition, (time - pivotStartTime) / pivotDuration));
        }

        if (liftActive) {
            if (time - liftStartTime > liftDuration)
                liftActive = false;

            lift.setPositionInch(smoothLerp(liftStartPosition, liftEndPosition, (time - liftStartTime) / liftDuration));
        }

    }

    void moveGrabberToPose() {
        wrist.wristServo1.setAngle(pose.targetGrabberAngle - pivot.getTargetPositionDeg());
    }

    public void hingeWristAuto(double deltaTime) {
        pose.targetGrabberAngle += settings.getWrist1Stick() * settings.wrist1Sensitivity * deltaTime;

        //pose.targetGrabberAngle = MathUtils.clamp(pivot.getTargetPositionDeg(), pivot.getTargetPositionDeg() - wrist.wristServo1.maxAngleDeg, pivot.getTargetPositionDeg() + wrist.wristServo1.maxAngleDeg); // clamps the pose within the achievable range to make the driver experience better

        switch (wrist.activeGrabber) {
            case groundGrabber:
                //pose.targetGrabberAngle = MathUtils.clamp(pose.targetGrabberAngle, 30, 110);
                break;
            case intakeGrabber:
                pose.targetGrabberAngle = MathUtils.clamp(pose.targetGrabberAngle, 100, 392);
                break;
        }

        moveGrabberToPose();

        opMode.telemetry.addData("target Wrist angle", "pose %f,  wrist angle %f", pose.targetGrabberAngle, wrist.wristServo1.getAngle());
    }







}



