package org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;

public class MultiMotor implements DcMotorEx {

    List<DcMotorEx> motors;

    RunMode runMode = RunMode.RUN_WITHOUT_ENCODER;
    Direction direction = Direction.FORWARD;
    ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.FLOAT;
    double motorPower = 0;
    int targetPosition = 0;

    HardwareMap hardwareMap;

    public MultiMotor(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        motors = new ArrayList<DcMotorEx>();

    }

    public void addMotor(String deviceName, Direction direction) {
        motors.add(hardwareMap.get(DcMotorEx.class, deviceName));

        motors.get(motors.size() - 1).setDirection(direction);
        motors.get(motors.size() - 1).setTargetPosition(targetPosition);

        forceUpdateRunMode(runMode);
    }
    public void addMotor(DcMotorEx motor) {
        motors.add(motor);

        motors.get(motors.size() - 1).setDirection(direction);
        motors.get(motors.size() - 1).setTargetPosition(targetPosition);

        forceUpdateRunMode(runMode);
    }

    public DcMotorEx getMotor(String deviceName){
       return hardwareMap.get(DcMotorEx.class, deviceName);
    }


    @Override
    public void setMotorEnable() {
        for (DcMotorEx motor : motors)
            motor.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        for (DcMotorEx motor : motors)
            motor.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return false;
    }//TODO

    @Override
    public void setVelocity(double angularRate) {
        for (DcMotorEx motor : motors)
            motor.setVelocity(angularRate);
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        for (DcMotorEx motor : motors)
            motor.setVelocity(angularRate, unit);
    }

    @Override
    public double getVelocity() {
        double velocitySum = 0;
        int activeMotorCount = 0;

        for (DcMotorEx motor : motors)
            if (motor.isMotorEnabled()) {
                activeMotorCount++;
                velocitySum += motor.getVelocity();
            }
        return velocitySum / activeMotorCount;
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        double velocitySum = 0;
        int activeMotorCount = 0;

        for (DcMotorEx motor : motors)
            if (motor.isMotorEnabled()) {
                activeMotorCount++;
                velocitySum += motor.getVelocity(unit);
            }
        return velocitySum / activeMotorCount;
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        for (DcMotorEx motor : motors)
            motor.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        for (DcMotorEx motor : motors)
            motor.setPIDFCoefficients(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        for (DcMotorEx motor : motors)
            motor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        for (DcMotorEx motor : motors)
            motor.setPositionPIDFCoefficients(p);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return null;
    }//TODO

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return null;
    }//TODO

    @Override
    public void setTargetPositionTolerance(int tolerance) {//TODO

    }

    @Override
    public int getTargetPositionTolerance() {
        return 0;
    }//TODO

    @Override
    public double getCurrent(CurrentUnit unit) {
        double currentSum = 0;
        int activeMotorCount = 0;

        for (DcMotorEx motor : motors)
            if (motor.isMotorEnabled()) {
                activeMotorCount++;
                currentSum += motor.getCurrent(unit);
            }

        return currentSum / activeMotorCount;
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return 0;
    }//TODO

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {//TODO

    }

    @Override
    public boolean isOverCurrent() {
        return false;
    }//TODO

    @Override
    public MotorConfigurationType getMotorType() {
        return null;
    }//TODO

    @Override
    public void setMotorType(MotorConfigurationType motorType) {//TODO

    }

    @Override
    public DcMotorController getController() {
        return null;
    }//TODO

    @Override
    public int getPortNumber() {
        return 0;
    }//TODO

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        if (this.zeroPowerBehavior == zeroPowerBehavior)
            return;
        this.zeroPowerBehavior = zeroPowerBehavior;

        for (DcMotorEx motor : motors)
            motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    void forceUpdatePowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;

        for (DcMotorEx motor : motors)
            motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    @Override
    public void setPowerFloat() {//TODO

    }

    @Override
    public boolean getPowerFloat() {//TODO
        return false;
    }

    @Override
    public void setTargetPosition(int position) {//TODO
        setMode(RunMode.RUN_TO_POSITION);

        if (this.targetPosition == position)
            return;

        for (DcMotorEx motor : motors)
            motor.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return targetPosition;
    }

    @Override
    public boolean isBusy() {
        return false;
    }//TODO

    @Override
    public int getCurrentPosition() {
        int positionSum = 0;
        int activeMotorCount = 0;

        for (DcMotorEx motor : motors)
            if (motor.isMotorEnabled()) {
                activeMotorCount++;
                positionSum += motor.getCurrentPosition();
            }
        return (int) ((float) positionSum / (float) activeMotorCount);
    }


    @Override
    public void setMode(DcMotor.RunMode runMode) {
        if (this.runMode == runMode)
            return;

        this.runMode = runMode;
        for (DcMotorEx motor : motors)
            motor.setMode(runMode);
    }

    public void forceUpdateRunMode(DcMotor.RunMode runMode) {
        this.runMode = runMode;
        for (DcMotorEx motor : motors)
            motor.setMode(runMode);
    }

    @Override
    public RunMode getMode() {
        return runMode;
    }

    @Override
    public void setDirection(Direction direction) {
        if (direction == this.direction)
            return;

        for (DcMotorEx motor : motors)
            toggleDirection(motor);
    }

    void toggleDirection(DcMotor motor) {
        if (motor.getDirection() == Direction.FORWARD)
            motor.setDirection(Direction.REVERSE);
        else
            motor.setDirection(Direction.FORWARD);
    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    @Override
    public void setPower(double power) {
        motorPower = power;
        for (DcMotorEx motor : motors)
            motor.setPower(motorPower);
    }

    @Override
    public double getPower() {
        return motorPower;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }//TODO

    @Override
    public String getDeviceName() {
        return null;
    }//TODO

    @Override
    public String getConnectionInfo() {
        return null;
    }//TODO

    @Override
    public int getVersion() {
        return 0;
    }//TODO

    @Override
    public void resetDeviceConfigurationForOpMode() {//TODO
    }

    @Override
    public void close() {
        for (DcMotor motor : motors)
            motor.close();
    }
}
