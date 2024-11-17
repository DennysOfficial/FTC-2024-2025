package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Trajectories;

public class MotionState {

    double position = 0;

    double velocity = 0;

    double acceleration = 0;

    double jerk = 0;

    public MotionState(double position, double velocity, double acceleration, double jerk){
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = jerk;
    }
    public MotionState(double position, double velocity, double acceleration){
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = 0;
    }
    public MotionState(double position, double velocity){
        this.position = position;
        this.velocity = velocity;
        this.acceleration = 0;
        this.jerk = 0;
    }
    public MotionState(double position){
        this.position = position;
        this.velocity = 0;
        this.acceleration = 0;
        this.jerk = 0;
    }

}
