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

package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes.BasicMechanumDrive;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes.DriveModeBase;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftPivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.RightLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.RightPivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.PassiveGrabber;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.speedyServos;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.StopWatch;

import java.util.List;

@TeleOp(name = "The One And Only Passive Grabber", group = "Az cringe")
//@Disabled
public class TheOneAndOnlyPassiveGrabber extends LinearOpMode {


    private final ElapsedTime frameTimer = new ElapsedTime();

    public boolean yPre = false;
    public boolean rTrigger = false;
    public boolean gotSample = false;
    public boolean wait = false;
    StopWatch stopWatch = new StopWatch();

    @Override
    public void runOpMode() {


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk// bulk caching and ftc telemetry

        RobotConfig activeConfig = new RobotConfig(this); // selects the active setting that will be used in the rest of the code


        DriveModeBase activeDriveMode = new BasicMechanumDrive(this, activeConfig);


        RightLift rightLift = new RightLift(ControlAxis.ControlMode.gamePadVelocityControl, this, activeConfig);

        RightPivot spinnyBit = new RightPivot(ControlAxis.ControlMode.gamePadVelocityControl, this, activeConfig);

        spinnyBit.assignLift(rightLift);
        rightLift.assignPivot(spinnyBit);

        LeftLift leftLift = new LeftLift(ControlAxis.ControlMode.positionControl, this, activeConfig);

        LeftPivot otherSpinnyBit = new LeftPivot(ControlAxis.ControlMode.positionControl, this, activeConfig);

        otherSpinnyBit.assignLift(leftLift);
        leftLift.assignPivot(otherSpinnyBit);

        PassiveGrabber leftArmStuff = new PassiveGrabber(this,activeConfig,leftLift,otherSpinnyBit);


        speedyServos prayers = new speedyServos(this, activeConfig);


        waitForStart();
        frameTimer.reset();
        //leftArmStuff.Rest();

        double deltaTime = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            stopWatch.reset();
            stopWatch.debug = activeConfig.debugConfig.getTimeBreakdownDebug();


            deltaTime = frameTimer.seconds(); //gets the time since the start of last frame and then resets the timer
            telemetry.addData("deltaTime", deltaTime);
            frameTimer.reset();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            activeConfig.sensorData.update(); // bulk caching



            if (gamepad1.y && !gotSample) {
                if (!spinnyBit.isBusy() && spinnyBit.getPosition() < 60)
                    spinnyBit.fancyMoveToPosition(65, 1);
                if (!rightLift.isBusy() && spinnyBit.getPosition() < 60)
                    rightLift.fancyMoveToPosition(0, 0.75);
                yPre = true;
                prayers.subStuff(1);
            }
            if (gamepad1.y && gotSample){
                if (!spinnyBit.isBusy())
                    spinnyBit.fancyMoveToPosition(76.5, 0.3);
            }

            if (yPre && !spinnyBit.isBusy() && !rightLift.isBusy())
            {

                rightLift.fancyMoveToPosition(13 ,0.5);
                spinnyBit.fancyMoveToPosition(76, 0.5);
                yPre = false;
            }


            if (gamepad1.b) {
                if (!spinnyBit.isBusy())
                    spinnyBit.fancyMoveToPosition(-69, 1);
                if (!rightLift.isBusy())
                    rightLift.fancyMoveToPosition(0, 0.75);
            }// presets

            if(gamepad1.x){
                leftArmStuff.Score();
            }
            if(gamepad1.a){
                leftArmStuff.Collect();
            }
            if (gamepad1.left_bumper){
                prayers.Intake();
                gotSample = false;
            }

//            // make the arm smack into the ground and intake
            if (spinnyBit.getControlMode() != ControlAxis.ControlMode.disabled && !spinnyBit.isBusy() && spinnyBit.getPosition() > 55) {
                if (gamepad1.right_trigger > 0.2){
                    spinnyBit.fancyMoveToPosition(83, 0.5);
                }
                if (prayers.isHold(gamepad1.right_trigger)){
                    spinnyBit.setTargetPosition(88);
                    rTrigger = true;
                }
            }
            if (!spinnyBit.isBusy() && rTrigger){
                if (spinnyBit.getPosition() > 85.3){
                    spinnyBit.fancyMoveToPosition(87, 0.5);
                    rTrigger = false;
                    gotSample = true;
                    wait = true;
                }
            }
            if (!spinnyBit.isBusy() && wait){
                if (!spinnyBit.isBusy())
                    spinnyBit.fancyMoveToPosition(-69, 1);
                if (!rightLift.isBusy())
                    rightLift.fancyMoveToPosition(0, 0.75);
                wait = false;
            }
//                spinnyBit.setControlMode(ControlAxis.ControlMode.gamePadTorqueControl);
//                spinnyBit.targetTorque = (gamepad2.right_trigger * activeConfig.sensitivities.getMaxGoDownAmount());
//
//            } else if (spinnyBit.getControlMode() == ControlAxis.ControlMode.gamePadTorqueControl)
//                spinnyBit.setControlModeUnsafe(spinnyBit.defaultControlMode); //



            stopWatch.addTimeToTelemetryAndReset(telemetry, "main loop beginning Time -------------------------------");

            rightLift.update();
            stopWatch.addTimeToTelemetryAndReset(telemetry, "main loop lift update Time -----------------------------");

            spinnyBit.update();
            stopWatch.addTimeToTelemetryAndReset(telemetry, "main loop pivot update Time ----------------------------");

            activeDriveMode.updateDrive(deltaTime);
            stopWatch.addTimeToTelemetryAndReset(telemetry, "main loop drive update Time ----------------------------");

            leftLift.update();
            otherSpinnyBit.update();
            prayers.update();


            telemetry.update();
        }
    }

}
