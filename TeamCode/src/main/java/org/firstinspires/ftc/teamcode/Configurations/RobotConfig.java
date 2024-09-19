package org.firstinspires.ftc.teamcode.Configurations;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
public class RobotConfig {

    LinearOpMode opMode;
    
    public static class MotorNames{
        public String frontRightDrive = "FR";
        public String frontLeftDrive = "FL";
        public String backRightDrive = "BR";
        public String backLeftDrive = "BL";
        
        public String rightPivotMotor = "PivotR";
        public String leftPivotMotor = "PivotL";

        public String rightLiftMotor = "LiftR";
        public String leftLiftMotor = "LiftL";

    }

    public RobotConfig(LinearOpMode opModeTemp) {
        opMode = opModeTemp;
    }

    static float driveSensitivity = 1;
    public float getDriveSensitivity() {
        return driveSensitivity;
    }

    static float forwardSensitivity = 1; // basic driving sensitivities only relative to each other
    public float getForwardSensitivity(){
        return forwardSensitivity;
    }

    static float turningSensitivity = 1;
    public float getTurningSensitivity(){
        return turningSensitivity;
    }

    static float strafingSensitivity = 1;
    public float getStrafingSensitivity(){
        return strafingSensitivity;
    }

    static float liftRate = 3; // inches per second
    public float getLiftRate() {
        return liftRate;
    }

    static float pivotRate = 20; //degrees per second
    public float getPivotRate(){
        return pivotRate;
    }

    public double getForwardStick() {
        return -1 * opMode.gamepad1.left_stick_y;
    } // button mapping

    public double getStrafeStick() {
        return opMode.gamepad1.left_stick_x;
    }

    public double getTurnStick() {
        return opMode.gamepad1.right_stick_x;
    }

    public double getLiftStick() {
        return opMode.gamepad2.left_stick_y;
    }

    public double getPivotStick() {
        return opMode.gamepad2.left_stick_x;
    }

    public boolean getIntakeInButton() {
        return opMode.gamepad1.right_bumper;
    }

    public boolean getIntakeOutButton() {
        return opMode.gamepad1.left_bumper;
    }

}
