package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings;


import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.OldLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.OldPivot;

import java.util.ArrayList;
import java.util.List;

public class Animator {

    ReadOnlyRuntime runtime;
    OpMode opMode;
    RobotConfig config;

    OldPivot spinyBit;
    OldLift oldLift;

    double startPosition;
    double endPosition;
    double duration;
    double startTime;

    boolean busy = false;

    class animation{
        double startTime;
        double endTime;
        double startPosition;
        double endPosition;
    }

    List<animation> animationQueue = new ArrayList<>();


    public Animator(ReadOnlyRuntime runtime, OpMode opMode, RobotConfig config, OldPivot spinyBit, OldLift oldLift) {
        this.runtime = runtime;
        this.opMode = opMode;
        this.config = config;

        this.spinyBit = spinyBit;
        this.oldLift = oldLift;
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

    public void abort() {
        busy = false;
    }

}



