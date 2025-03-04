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
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes.NextFTCDrive;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.armGroup;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Lift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Pivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.PassiveGrabberGroup;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.StopWatch;

import java.util.List;

@TeleOp(name = "Arm Group Test", group = "bb - testing") //brrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr
//@Disabled
public class ArmGroupTest extends LinearOpMode {

    public static boolean driver2PresetsEnabled = false;

    private final ElapsedTime frameTimer = new ElapsedTime();

    StopWatch stopWatch = new StopWatch();

    IMU imu;

    @Override
    public void runOpMode() {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk


        RobotConfig activeConfig = new RobotConfig(this); // selects the active setting that will be used in the rest of the code

        assert activeConfig.playerOne != null; // i don't like yellow lines
        assert activeConfig.playerTwo != null;

        NextFTCDrive vroom = new NextFTCDrive(this, activeConfig);

        armGroup arm = new armGroup(
                this,
                activeConfig,
                new Lift(ControlAxis.ControlMode.gamePadVelocityControl, this, activeConfig),
                new Pivot(ControlAxis.ControlMode.gamePadVelocityControl, this, activeConfig),
                "main"
        );


        PassiveGrabberGroup grabber = new PassiveGrabberGroup(this, activeConfig, arm);


        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection. RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));


        waitForStart();
        frameTimer.reset();

        double deltaTime = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            stopWatch.reset();
            stopWatch.debug = activeConfig.debugConfig.getTimeBreakdownDebug();


            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);


            deltaTime = frameTimer.seconds(); //gets the time since the start of last frame and then resets the timer
            telemetry.addData("deltaTime", deltaTime);
            frameTimer.reset();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            activeConfig.sensorData.update();


            if (gamepad2.x && driver2PresetsEnabled) {
                arm.fancyMoveArm(12.5, 1, 16, 1);
            }

            if (gamepad2.y && driver2PresetsEnabled) {
                if (arm.getLiftPos() < 15)
                    arm.fancyMovePivot(-18, 1);


                if (arm.getPivotPos() < 50)
                    arm.setTargetPosLift(33);

            }


            if (gamepad2.a && driver2PresetsEnabled) {
                if (arm.getLiftPos() > 25 && arm.getPivotPos() < -5)
                    arm.fancyMovePivot(0, 1);
                arm.setTargetPosLift(0);
                if (arm.getLiftPos() < 14)
                    arm.fancyMovePivot(71, 1);
            }


            if (activeConfig.playerOne.grabberScore.getState()) {
                grabber.Score();
            }

            if (activeConfig.playerOne.grabberCollect.getState()) {
                grabber.Collect();
            }

            if (activeConfig.playerOne.grabberRest.getState()) {
                grabber.Rest();
            }



            // make the arm smack into the ground and intake
            if (arm.getPivotControlMode() != ControlAxis.ControlMode.disabled && !arm.isPivotBusy() && gamepad2.right_trigger > 0.2 && arm.getPivotPos() > 60) {

                arm.setPivotControlMode(ControlAxis.ControlMode.gamePadTorqueControl, true);
                arm.setPivotTargetTorque(gamepad2.right_trigger * activeConfig.sensitivities.getMaxGoDownAmount());

            } else if (arm.getPivotControlMode() == ControlAxis.ControlMode.gamePadTorqueControl)
                arm.setPivotControlMode(arm.pivotDefaultMode(), false);

            stopWatch.addTimeToTelemetryAndReset(telemetry, "main loop beginning Time -------------------------------");

            arm.update();
            stopWatch.addTimeToTelemetryAndReset(telemetry, "main loop lift update Time -----------------------------");
            stopWatch.addTimeToTelemetryAndReset(telemetry, "main loop pivot update Time ----------------------------");

            vroom.updateDrive(deltaTime);
            stopWatch.addTimeToTelemetryAndReset(telemetry, "main loop drive update Time ----------------------------");

            telemetry.addData("yaw (heading)", "%.2f deg", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("pitch", "%.2f deg", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("roll", "%.2f deg\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("yaw velocity", "%.2f deg/sec", angularVelocity.zRotationRate);
            telemetry.addData("pitch velocity", "%.2f deg/sec", angularVelocity.xRotationRate);
            telemetry.addData("roll velocity", "%.2f deg/sec", angularVelocity.yRotationRate);

            activeConfig.playerOne.update_all();
            activeConfig.playerTwo.update_all();

            telemetry.update();
        }
    }

}
