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

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.CustomPID;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.PositionDerivatives;

@TeleOp(name = "Intake Test", group = "Linear OpMode")
//@Disabled
public class IntakeTestingThing extends LinearOpMode {


    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     * that can throw {@link InterruptedException}
     */
    DcMotorEx motor1;
    DcMotorEx motor2;
    private final ElapsedTime frameTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk// bulk caching and ftc telemetry

        RobotConfig config = new RobotConfig(this);

        motor1 = hardwareMap.get(DcMotorEx.class, config.deviceConfig.rightLift);
        motor2 = hardwareMap.get(DcMotorEx.class, config.deviceConfig.rightPivot);

        motor2.setDirection(DcMotorSimple.Direction.REVERSE);


        CustomPID motor1VelocityPID = new CustomPID(telemetry, config, "motor1VelocityPID");
        CustomPID motor2VelocityPID = new CustomPID(telemetry, config, "motor2VelocityPID");

        double motor1Pos = motor1.getCurrentPosition();
        double motor2Pos = motor2.getCurrentPosition();

        PositionDerivatives motor1PDs = new PositionDerivatives(motor1Pos * IntakeTestConfig.motor1GearRatio);
        PositionDerivatives motor2PDs = new PositionDerivatives(motor2Pos * IntakeTestConfig.motor2GearRatio);


        double motor1Power = 0;
        double motor2Power = 0;

        double deltaTime = 0;

        waitForStart();
        frameTimer.reset();

        while (opModeIsActive()) {

            motor1.setPower(motor1Power);
            motor2.setPower(motor2Power);

            telemetry.addData("motor1 Target Velocity", IntakeTestConfig.motor1TargetVelocityRPM);
            telemetry.addData("motor1 Actual Velocity", motor1PDs.getVelocity());
            telemetry.addData("motor1 power", motor1Power);
            telemetry.addLine();

            telemetry.addData("motor2 Target Velocity", IntakeTestConfig.motor2TargetVelocityRPM);
            telemetry.addData("motor2 Actual Velocity", motor2PDs.getVelocity());
            telemetry.addData("motor2 power", motor2Power);
            telemetry.update();

            deltaTime = frameTimer.seconds(); //gets the time since the start of last frame and then resets the timer
            telemetry.addData(" deltaTime", deltaTime);
            telemetry.addLine();
            frameTimer.reset();

            motor1Pos = motor1.getCurrentPosition();
            motor2Pos = motor2.getCurrentPosition();

            motor1PDs.update(motor1Pos * IntakeTestConfig.motor1GearRatio, deltaTime);
            motor2PDs.update(motor2Pos * IntakeTestConfig.motor2GearRatio, deltaTime);

            if (config.inputMap == null || !config.inputMap.getIntakeButton()) {
                motor1Power = 0;
                motor2Power = 0;
                continue;
            }

            motor1VelocityPID.setCoefficients(IntakeTestConfig.motor1Kp, IntakeTestConfig.motor1Ki, IntakeTestConfig.motor1Kd);
            motor2VelocityPID.setCoefficients(IntakeTestConfig.motor2Kp, IntakeTestConfig.motor2Ki, IntakeTestConfig.motor2Kd);

            motor1Power = motor1VelocityPID.runPID(IntakeTestConfig.motor1TargetVelocityRPM, motor1PDs.getVelocity(), deltaTime);
            motor2Power = motor2VelocityPID.runPID(IntakeTestConfig.motor2TargetVelocityRPM, motor2PDs.getVelocity(), deltaTime);


        }
    }
}
