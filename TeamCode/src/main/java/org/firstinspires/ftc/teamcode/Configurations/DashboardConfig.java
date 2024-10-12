package org.firstinspires.ftc.teamcode.Configurations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class DashboardConfig extends RobotConfig {

    DashboardConfig(LinearOpMode opMode, RobotConfig config){
        super(opMode);
    }

    @Override
    public double getForwardStick() {
        return opMode.gamepad1.right_stick_x;
    } // button mapping
    @Override
    public double getStrafeStick() {
        return -1 * opMode.gamepad1.right_stick_y;
    }
    @Override
    public double getTurnStick() {
        return opMode.gamepad1.left_stick_y;
    }
    @Override
    public double getLiftStick() {
        return opMode.gamepad2.right_stick_x;
    }
    @Override
    public double getPivotStick() {
        return -1 * opMode.gamepad2.left_stick_x;
    }
    



}
