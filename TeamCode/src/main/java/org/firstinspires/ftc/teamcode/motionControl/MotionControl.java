package org.firstinspires.ftc.teamcode.motionControl;


import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.MathStuff;

public class MotionControl {

    ElapsedTime runtime;
    OpMode opMode;
    RobotConfig config;

    double startPosition;
    double endPosition;
    double duration;
    double startTime;

    boolean busy = false;


    public MotionControl(ElapsedTime runtime, OpMode opMode, RobotConfig config) {
        this.runtime = runtime;
        this.opMode = opMode;
        this.config = config;
    }

    /**
     * linearly interpolates between point 1 and point 2 by interpolation amount
     *
     * @param interpolationAmount normally ranges from 0-1
     */

    /**
     * sinusoidally interpolates between point 1 and point 2 by interpolation amount
     *
     * @param interpolationAmount clamped from 0-1
     */
    double smoothLerp(double point1, double point2, double interpolationAmount) {
        interpolationAmount *= Math.sin(interpolationAmount * Math.PI / 2.0);
        interpolationAmount = MathUtils.clamp(interpolationAmount, 0, 1);
        return MathStuff.lerp(point1, point2, interpolationAmount);
    }

    public void smoothMove(double startPosition, double endPosition, double duration) {
        if (busy)
            return;

        busy = true;

        this.startPosition = startPosition;
        this.endPosition = endPosition;
        this.duration = duration;
        this.startTime = runtime.seconds();
    }

    public double update() {
        if (!busy)
            return Double.NaN;

        if (runtime.seconds() - startTime > duration)
            busy = false;

        return smoothLerp(startPosition, endPosition, (runtime.seconds() - startTime) / duration);
    }

    public boolean isBusy() {
        return busy;
    }


}



