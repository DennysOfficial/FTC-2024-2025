package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotionState {

    public double position = 0;

    public double velocity = 0;

    public double acceleration = 0;


    public MotionState(double position, double velocity, double acceleration) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    public MotionState(double position, double velocity) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = 0;
    }

    public MotionState(double position) {
        this.position = position;
        this.velocity = 0;
        this.acceleration = 0;
    }

    public MotionState() {
        this.position = Double.NaN;
        this.velocity = 0;
        this.acceleration = 0;
    }

    public static void telemetryMotionState(Telemetry telemetry, MotionState motionState, String name) {
        telemetry.addData(name + " position", motionState.position);
        telemetry.addData(name + " velocity", motionState.velocity);
        telemetry.addData(name + " acceleration", motionState.acceleration);
    }

}
