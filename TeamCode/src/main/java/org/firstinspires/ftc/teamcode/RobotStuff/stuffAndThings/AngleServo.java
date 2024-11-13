package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class AngleServo implements Servo {
    double scalePosUnitPerDeg;
    double offsetPosUnit;

    Servo baseServo;

    /**
     * given two points and corresponding measured angles this class allows a servo to be set to an angle rather than position
     */
    public AngleServo(String deviceName, HardwareMap hardwareMap, double point1Position, double point1Angle, double point2Position, double point2Angle) {
        baseServo = hardwareMap.get(Servo.class, deviceName);

        calcScaleAndOffset(point1Position,point1Angle,point2Position,point2Angle);
    }

    public void calcScaleAndOffset(double point1Position, double point1Angle, double point2Position, double point2Angle) {
        scalePosUnitPerDeg = (point1Position - point2Position) / (point1Angle - point2Angle);
        offsetPosUnit = point1Position - scalePosUnitPerDeg * point1Angle;
    }

    double angleToPosition(double angle) {
        return angle * scalePosUnitPerDeg + offsetPosUnit;
    }

    double positionToAngle(double position) {
        return (position - offsetPosUnit) / scalePosUnitPerDeg;
    }

    public void setAngle(double angle) {
        setPosition(angleToPosition(angle));
    }

    public double getAngle() {
        return positionToAngle(getPosition());
    }

    @Override
    public ServoController getController() {
        return baseServo.getController();
    }

    @Override
    public int getPortNumber() {
        return baseServo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        baseServo.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return baseServo.getDirection();
    }

    @Override
    public void setPosition(double position) {
        baseServo.setPosition(position);
    }

    @Override
    public double getPosition() {
        return baseServo.getPosition();
    }

    /**
     * DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS DO NOT USE THIS
     */
    @Override
    public void scaleRange(double min, double max) {

    }

    @Override
    public Manufacturer getManufacturer() {
        return baseServo.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return baseServo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return baseServo.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return baseServo.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        baseServo.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        baseServo.close();
    }
}
