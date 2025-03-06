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
import org.firstinspires.ftc.teamcode.RobotStuff.SampleArm;
import org.firstinspires.ftc.teamcode.RobotStuff.SampleArmPose;
import org.firstinspires.ftc.teamcode.RobotStuff.SpecimenArm;
import org.firstinspires.ftc.teamcode.RobotStuff.SpecimenArmPose;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes.BasicMechanumDrive;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes.DriveModeBase;

import java.util.List;

@TeleOp(name = "full test", group = "AB important Testing / main opMode ")
//@Disabled
public class clawAngleIncrementTest extends LinearOpMode {

    boolean hangStarted = false;

    int hangStage = 0;
    private final ElapsedTime frameTimer = new ElapsedTime();

    SampleArm sampleArm;

    SpecimenArm specimenArm;

    void hang() {
        switch (hangStage) {
            case 0:
                sampleArm.moveToPose(new SampleArmPose(0, 0.420, 0, 0), 1);
                specimenArm.moveToPose(new SpecimenArmPose(0.73, 0, 0), 1);
                hangStage = 1;
                break;
            case 1:
                if (!specimenArm.isBusy() &&  !sampleArm.isBusy()) {
                    sampleArm.moveToPose(new SampleArmPose(0, 0.420, 17, 0), 1);
                    specimenArm.moveToPose(new SpecimenArmPose(0.73, 17, 0), 1);
                    hangStage = 2;
                }
                break;
            case 2:
                if (!specimenArm.isBusy() &&  !sampleArm.isBusy()) {
                    sampleArm.moveToPose(new SampleArmPose(0, 0.420, 17, -40), 0.7);
                    specimenArm.moveToPose(new SpecimenArmPose(0.73, 17, -40), 0.7);
                    hangStage = 3;
                }
                break;
            case 3:
                if (!specimenArm.isBusy() &&  !sampleArm.isBusy()) {
                    sampleArm.moveToPose(new SampleArmPose(0, 0.420, 5, -40), 4);
                    specimenArm.moveToPose(new SpecimenArmPose(0.73, 5, -40), 4);
                    hangStage = 4;
                }
                break;
        }
    }


    @Override
    public void runOpMode() {


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk// bulk caching and ftc telemetry

        RobotConfig activeConfig = new RobotConfig(this); // selects the active setting that will be used in the rest of the code


        DriveModeBase activeDriveMode = new BasicMechanumDrive(this, activeConfig);

        sampleArm = new SampleArm(this, activeConfig);

        specimenArm = new SpecimenArm(this, activeConfig);


        waitForStart();
        frameTimer.reset();
        //leftArmStuff.Rest();

        double deltaTime = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            activeConfig.stopWatch.reset();
            activeConfig.stopWatch.debug = activeConfig.debugConfig.getTimeBreakdownDebug();


            deltaTime = frameTimer.seconds(); //gets the time since the start of last frame and then resets the timer
            telemetry.addData("deltaTime", deltaTime);
            frameTimer.reset();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            activeConfig.sensorData.update(); // bulk caching

            if (gamepad1.dpad_down && gamepad2.dpad_down) {
                hang();
            }



            activeConfig.stopWatch.addTimeToTelemetryAndReset(telemetry, "main loop beginning Time -------------------------------");

            sampleArm.update();
            activeConfig.stopWatch.addTimeToTelemetryAndReset(telemetry, "total HarpoonArm update Time ----------------------------");

            specimenArm.update();
            activeConfig.stopWatch.addTimeToTelemetryAndReset(telemetry, "total SpecimenArm update Time ----------------------------");

            activeDriveMode.updateDrive(deltaTime);
            activeConfig.stopWatch.addTimeToTelemetryAndReset(telemetry, "drive update Time ----------------------------");

            telemetry.update();
        }
    }

}
