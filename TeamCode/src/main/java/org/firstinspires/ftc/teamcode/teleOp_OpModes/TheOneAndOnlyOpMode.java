/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleOp_OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.individual_components.DriveModes.BasicMechanumDrive;
import org.firstinspires.ftc.teamcode.individual_components.DriveModes.DriveModeBase;
import org.firstinspires.ftc.teamcode.individual_components.Lift;
import org.firstinspires.ftc.teamcode.individual_components.Pivot;
import org.firstinspires.ftc.teamcode.individual_components.grabbers.ActiveIntake;
import org.firstinspires.ftc.teamcode.motionControl.Animator;

import java.util.List;


@TeleOp(name = "The One and Only OpMode", group = "Linear OpMode")
//@Disabled
public class TheOneAndOnlyOpMode extends LinearOpMode {


    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime frameTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk


        RobotConfig activeConfig = new RobotConfig(this); // selects the active setting that will be used in the rest of the code


        DriveModeBase activeDriveMode = new BasicMechanumDrive(this, activeConfig);

        
        Lift lift = new Lift(this, activeConfig);

        final ControlAxis.ControlMode defaultLiftControlMode = ControlAxis.ControlMode.positionControl;

        lift.setControlMode(defaultLiftControlMode);

        Pivot spinnyBit = new Pivot(this, activeConfig);

        final ControlAxis.ControlMode defaultPivotControlMode = ControlAxis.ControlMode.positionControl;

        spinnyBit.setControlMode(defaultPivotControlMode);


        //Pincher pincher = new Pincher(this,activeConfig);

        ActiveIntake intake = new ActiveIntake(this, activeConfig);


        Animator pivotControl = new Animator(runtime, this, activeConfig, spinnyBit, lift);


        waitForStart();
        runtime.reset();
        frameTimer.reset();

        double deltaTime = 0;

        double predictedPivotTargetPosition = spinnyBit.getTargetPosition();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("pivotMotorR", hardwareMap.get(DcMotorEx.class, activeConfig.deviceConfig.rightPivot).getCurrentPosition());
            telemetry.addData("pivotMotorL", hardwareMap.get(DcMotorEx.class, activeConfig.deviceConfig.leftPivot).getCurrentPosition());


            if (activeConfig.inputMap.getUnAbort()) {
                lift.setControlMode(defaultLiftControlMode);
                spinnyBit.setControlMode(defaultPivotControlMode);
            }

            deltaTime = frameTimer.seconds(); //gets the time since the start of last frame and then resets the timer
            telemetry.addData("deltaTime", deltaTime);
            frameTimer.reset();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            activeConfig.sensorData.update();

            if (gamepad2.x) {
                lift.setTargetPosition(0);
                if (lift.getPosition() < 15)
                    pivotControl.smoothMove(spinnyBit.getPosition(), -10, 1);

            }

            if (gamepad2.y) {
                if (lift.getPosition() < 10)
                    pivotControl.smoothMove(spinnyBit.getPosition(), -18, 1);


                if (spinnyBit.getPosition() < 40)
                    lift.setTargetPosition(31);

            }

            if (gamepad2.a) {
                if (lift.getPosition() > 25 && spinnyBit.getPosition() < -5)
                    pivotControl.smoothMove(spinnyBit.getPosition(), 0, 0.5);
                lift.setTargetPosition(0);
                if (lift.getPosition() < 14)
                    pivotControl.smoothMove(spinnyBit.getPosition(), 71, 1);
            }

            if (pivotControl.isBusy())
                spinnyBit.setTargetPosition(pivotControl.update());

            if (pivotControl.isBusy() && Math.abs(activeConfig.inputMap.getPivotStick()) > activeConfig.getAutoAbortThreshold()) {
                pivotControl.abort();
                spinnyBit.setTargetPosition(spinnyBit.getPosition());
            }

            if (spinnyBit.getControlMode() != ControlAxis.ControlMode.disabled && !pivotControl.isBusy() && gamepad2.right_trigger > 0.2 && spinnyBit.getPosition() > 60) {

                spinnyBit.setControlMode(ControlAxis.ControlMode.testing);
                spinnyBit.setNetTorque(gamepad2.right_trigger * activeConfig.sensitivities.getMaxGoDownAmount());

            } else if (spinnyBit.getControlMode() != ControlAxis.ControlMode.disabled)
                spinnyBit.setControlModeUnsafe(defaultPivotControlMode);

            lift.update(deltaTime, spinnyBit.getPosition());
            spinnyBit.update(deltaTime, lift.getPosition());
            activeDriveMode.updateDrive(deltaTime);

            intake.directControl();

            telemetry.addData("Run Time: ", runtime.toString());
            telemetry.update();
        }
    }

}
