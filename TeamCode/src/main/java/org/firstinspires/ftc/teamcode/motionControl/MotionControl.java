package org.firstinspires.ftc.teamcode.motionControl;


import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Configurations.RobotConfig;

public class MotionControl {

    ElapsedTime runtime;
    OpMode opMode;
    RobotConfig config;

    double startPosition;
    double endPosition;
    double duration;
    double startTime;

    boolean active = false;



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

    public void smoothMove(double startPosition, double endPosition, double duration) {

        if (active)
            return;

        active = true;

        this.startPosition = startPosition;
        this.endPosition = endPosition;
        this.duration = duration;
        this.startTime = runtime.seconds();

    };

    public double update(){
        if(!active)
            return 0;
        if(runtime.seconds()-startTime > duration)
            active = false;
        return smoothLerp(startPosition,endPosition,(runtime.seconds()-startTime)/duration);
    }

    public boolean isActive(){
        return active;
    }








}



