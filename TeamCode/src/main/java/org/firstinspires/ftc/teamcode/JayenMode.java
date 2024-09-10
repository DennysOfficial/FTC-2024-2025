package org.firstinspires.ftc.teamcode;

import android.os.Debug;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "JayenMode")
public class JayenMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //initialization code goes here

        DcMotor FLWheel;
        FLWheel = hardwareMap.get(DcMotor.class, "FLWheel"); //this needs to be changed to match the robot's motor configuration
        DcMotor FRWheel;
        FRWheel = hardwareMap.get(DcMotor.class, "FRWheel");
        DcMotor BRWheel;
        BRWheel = hardwareMap.get(DcMotor.class, "BRWheel");
        DcMotor BLWheel;
        BLWheel = hardwareMap.get(DcMotor.class, "BLWheel");

        FLWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        FRWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        BRWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        BLWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        FLWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart(); //after init, code waits for the start button

        while(opModeIsActive()){

            double forwardBackward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;

            double FLPower = forwardBackward - strafe*-1 - yaw;
            double FRPower = forwardBackward + strafe*-1 + yaw;
            double BLPower = forwardBackward + strafe*-1 - yaw;
            double BRPower = forwardBackward - strafe*-1 + yaw;

            FLWheel.setPower(FLPower);
            FRWheel.setPower(FRPower);
            BLWheel.setPower(BLPower);
            BRWheel.setPower(BRPower);
        }
    }
}

