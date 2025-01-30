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

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes.BasicMechanumDrive;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes.DriveModeBase;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftPivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.PassiveGrabber;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.StopWatch;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.InertialMeasurementUnit;

import java.util.List;

@TeleOp(name = "Motor Go Vrooom", group = "Linear OpMode")
//@Disabled
public class motorGoVrooom extends LinearOpMode { // this is basically theotheroneandonlyopmode but with imu stuff


    private final ElapsedTime frameTimer = new ElapsedTime();

    StopWatch stopWatch = new StopWatch();

    InertialMeasurementUnit IMU = new InertialMeasurementUnit();

    @Override
    public void runOpMode() {


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk// bulk caching and ftc telemetry

        RobotConfig activeConfig = new RobotConfig(this); // selects the active setting that will be used in the rest of the code


        DriveModeBase activeDriveMode = new BasicMechanumDrive(this, activeConfig);


        //RightLift rightLift = new RightLift(ControlAxis.ControlMode.gamePadVelocityControl, this, activeConfig);

        //RightPivot spinnyBit = new RightPivot(ControlAxis.ControlMode.gamePadVelocityControl, this, activeConfig);

        //spinnyBit.assignLift(rightLift);
        //rightLift.assignPivot(spinnyBit);

        LeftLift leftLift = new LeftLift(ControlAxis.ControlMode.gamePadVelocityControl, this, activeConfig);

        LeftPivot otherSpinnyBit = new LeftPivot(ControlAxis.ControlMode.gamePadVelocityControl, this, activeConfig);

        otherSpinnyBit.assignLift(leftLift);
        leftLift.assignPivot(otherSpinnyBit);

        PassiveGrabber dohicky = new PassiveGrabber(this,activeConfig,leftLift,otherSpinnyBit);

        //ActiveIntakeServo intake = new ActiveIntakeServo(this, activeConfig);

        IMU.initialize(hardwareMap);

        waitForStart();
        frameTimer.reset();
        IMU.afterStart(); // starts measuring

        double deltaTime = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            stopWatch.reset();
            stopWatch.debug = activeConfig.debugConfig.getTimeBreakdownDebug();

            IMU.updateIMU(telemetry);
            IMU.returnIMU(telemetry);

            deltaTime = frameTimer.seconds(); //gets the time since the start of last frame and then resets the timer
            telemetry.addData("deltaTime", deltaTime);
            frameTimer.reset();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            activeConfig.sensorData.update(); // bulk caching

            otherSpinnyBit.update();
            leftLift.update();

            if(gamepad2.b)
                dohicky.Collect();
            if(gamepad2.y)
                dohicky.Rest();
            if(gamepad2.x)
                dohicky.Score();

            activeDriveMode.updateDrive(deltaTime);
            stopWatch.addTimeToTelemetryAndReset(telemetry, "main loop drive update Time ----------------------------");

            //intake.directControl();

            telemetry.update();
        }
    }

}
