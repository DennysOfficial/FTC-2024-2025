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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.individual_components.DriveModes.BasicMechanumDrive;
import org.firstinspires.ftc.teamcode.individual_components.DriveModes.DriveModeBase;
import org.firstinspires.ftc.teamcode.individual_components.Lift;
import org.firstinspires.ftc.teamcode.individual_components.Pivot;
import org.firstinspires.ftc.teamcode.individual_components.grabbers.ActiveIntake;
import org.firstinspires.ftc.teamcode.motionControl.Animator;


@TeleOp(name = "AnotherOne: OpMode", group = "Linear OpMode")
//@Disabled
public class AnotherTestingOpMode extends LinearOpMode {


    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime frameTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); // if multiple sensors read calls are made in a row it will group them together making code more fasterer
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk


        RobotConfig activeConfig = new RobotConfig(this); // selects the active setting that will be used in the rest of the code


        DriveModeBase activeDriveMode = new BasicMechanumDrive(this, activeConfig);


        Lift lift = new Lift(this, activeConfig);

        lift.setControlMode(ControlAxis.ControlMode.directControl);

        Pivot spinyBit = new Pivot(this, activeConfig);

        spinyBit.setControlMode(ControlAxis.ControlMode.directControl);


        //Pincher pincher = new Pincher(this,activeConfig);

        ActiveIntake intake = new ActiveIntake(this, activeConfig);


        Animator pivotControl = new Animator(runtime, this, activeConfig, spinyBit, lift);


        waitForStart();
        runtime.reset();
        frameTimer.reset();

        double deltaTime = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            deltaTime = frameTimer.seconds(); //gets the time since the start of last frame and then resets the timer
            telemetry.addData("deltaTime ", deltaTime);
            frameTimer.reset();

            activeConfig.sensorData.update();

            if (gamepad2.x) {
                lift.setTargetPosition(0);
                if (lift.getPosition() < 15)
                    pivotControl.smoothMove(spinyBit.getPosition(), -10, 1);

            }

            if (gamepad2.y) {
                if (lift.getPosition() < 10)
                    pivotControl.smoothMove(spinyBit.getPosition(), -10, 1);


                if (spinyBit.getPosition() < 40)
                    lift.setTargetPosition(30);

            }

            if (gamepad2.a) {
                lift.setTargetPosition(0);
                if (lift.getPosition() < 14)
                    pivotControl.smoothMove(spinyBit.getPosition(), 75, 1);
            }

            if (pivotControl.isBusy())
                spinyBit.setTargetPosition(pivotControl.update());

            if (activeConfig.inputMap.getPivotStick() > activeConfig.getAutoAbortThreshold()) {
                pivotControl.abort();
                spinyBit.setTargetPosition(spinyBit.getPosition());
            }
            if (activeConfig.inputMap.getLiftStick() > activeConfig.getAutoAbortThreshold()) {
                lift.setTargetPosition(lift.getPosition());
            }


            lift.update(deltaTime, spinyBit.getPosition());
            spinyBit.update(deltaTime, lift.getPosition());
            activeDriveMode.updateDrive(deltaTime);

            intake.directControl();

            telemetry.addData("Run Time: ", runtime.toString());
            telemetry.update();
        }
    }

}
