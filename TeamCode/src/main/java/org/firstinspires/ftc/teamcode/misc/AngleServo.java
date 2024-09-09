package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;


public class AngleServo{

    public double minAngleDeg;
    public double maxAngleDeg;
    public double rangeDeg;
    LinearOpMode opMode;

    Servo servo;
    /**
     * allows servos to take an angle input

     * @param opMode        the opMode the servo is running in
     * @param servoName     the name of the servo in the active config
     * @param maxAngleDeg   the angle when position is set to 0
     * @param minAngleDeg   the angle when position is set to 1
     */
    public AngleServo(LinearOpMode opMode, String servoName, double minAngleDeg, double maxAngleDeg) {
        this.opMode = opMode;

        this.minAngleDeg = minAngleDeg;
        this.maxAngleDeg = maxAngleDeg;

        rangeDeg = maxAngleDeg - minAngleDeg;

        servo = opMode.hardwareMap.get(Servo.class, servoName);
    }

    public void setDirection(Servo.Direction direction){
        servo.setDirection(direction);
    }

    public double getAngle() {
        return servo.getPosition() * rangeDeg + minAngleDeg;
    }

    public void setAngle(double angle) {
        servo.setPosition((angle - minAngleDeg) / rangeDeg);
    }

}
