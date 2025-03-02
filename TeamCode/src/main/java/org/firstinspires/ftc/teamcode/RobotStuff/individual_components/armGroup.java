package org.firstinspires.ftc.teamcode.RobotStuff.individual_components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis.ControlMode;
import org.firstinspires.ftc.teamcode.teleOp_OpModes.ArmSwitch;

public class armGroup {

    protected OpMode opmode;
    protected RobotConfig config;

    protected ControlAxis lift;
    protected ControlAxis pivot;

    protected String name;

    public armGroup(OpMode opmode, RobotConfig config, Lift lift, Pivot pivot, String name) {
        this.config = config;
        this.opmode = opmode;
        this.lift = lift;
        this.pivot = pivot;
        this.name = name;

        lift.assignPivot(pivot);
        pivot.assignLift(lift);

    }

    public armGroup(OpMode opmode, RobotConfig config, LeftLift lift, LeftPivot pivot, String name) {
        this.config = config;
        this.opmode = opmode;
        this.lift = lift;
        this.pivot = pivot;
        this.name = name;

        lift.assignPivot(pivot);
        pivot.assignLift(lift);

    }

    public armGroup(OpMode opmode, RobotConfig config, RightLift lift, RightPivot pivot, String name) {
        this.config = config;
        this.opmode = opmode;
        this.lift = lift;
        this.pivot = pivot;
        this.name = name;

        lift.assignPivot(pivot);
        pivot.assignLift(lift);

    }


    ///////////////////////////////// fancy move /////////////////////////////////
    public void fancyMoveLift(double pos, double dur) {
        if (!lift.isBusy()) {lift.fancyMoveToPosition(pos, dur);}
    }

    public void fancyMovePivot(double pos, double dur) {
        if (!pivot.isBusy()) {pivot.fancyMoveToPosition(pos, dur);}
    }

    public void fancyMoveArm(double liftPos, double liftDur, double pivotPos, double pivotDur) {
        fancyMoveLift(liftPos, liftDur);
        fancyMovePivot(pivotPos, pivotDur);
    }

    ///////////////////////////////////target position/////////////////////////
    public void setTargetPosLift(double pos) {
        lift.setTargetPosition(pos);
    }

    public void setTargetPosPivot(double pos) {
        pivot.setTargetPosition(pos);
    }

    public void setTargetPosArm(double liftPos, double pivotPos) {
        setTargetPosLift(liftPos);
        setTargetPosPivot(pivotPos);
    }
    //////////////////////////////////// control mode //////////////////////////
    public void setPivotControlMode(ControlMode mode, boolean safe) {
       if (safe) {pivot.setControlMode(mode);} else {pivot.setControlModeUnsafe(mode);}
    }

    public void setLiftControlMode(ControlMode mode, boolean safe) {
        if (safe) {lift.setControlMode(mode);} else {lift.setControlModeUnsafe(mode);}
    }

    ///////////////////////////////// target stuff //////////////////////////
    public void setPivotTargetTorque(double targetTorque) {pivot.targetTorque = targetTorque;}




    public void update() {
        lift.update();
        pivot.update();
    }



    ///////////////////////non-void methods////////////////////////////
    public double getLiftPos() {return lift.getPosition();}

    public double getPivotPos() {return pivot.getPosition();}


    public boolean isPivotBusy() {return pivot.isBusy();}

    public boolean isLiftBusy() {return lift.isBusy();}


    public ControlMode getPivotControlMode() {return pivot.getControlMode();}

    public ControlMode getLiftControlMode() {return lift.getControlMode();}

    public ControlMode pivotDefaultMode() {return pivot.defaultControlMode;}

    public ControlMode liftDefaultMode() {return lift.defaultControlMode;}

    public double pivotTargetTorque() {return pivot.targetTorque;}
}
