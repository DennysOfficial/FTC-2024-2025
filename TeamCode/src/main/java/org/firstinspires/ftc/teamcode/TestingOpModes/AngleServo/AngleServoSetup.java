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

package org.firstinspires.ftc.teamcode.TestingOpModes.AngleServo;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.AngleServo;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.ButtonEdgeDetector;

@TeleOp(name = "Angle Servo Setup", group = "Abbc testing")
//@Disabled
public class AngleServoSetup extends LinearOpMode {


    double targetPosition = 0.5;
    private final ElapsedTime frameTimer = new ElapsedTime();

    ButtonEdgeDetector refreshServoButton = new ButtonEdgeDetector(false);

    AngleServo servo = null;

    @Override
    public void runOpMode() throws InterruptedException {


        double deltaTime = 0;

        waitForStart();
        while (opModeIsActive()) {
            deltaTime = frameTimer.seconds(); //gets the time since the start of last frame and then resets the timer
            //telemetry.addData(" deltaTime", deltaTime);
            frameTimer.reset();

            targetPosition += gamepad1.left_stick_y * deltaTime * AngleServoTestConfig.f_testingSensitivity;
            targetPosition = MathUtils.clamp(targetPosition, 0, 1);

            if (refreshServoButton.getButtonDown(gamepad1.a)) {
                try {
                    servo = new AngleServo(AngleServoTestConfig.a_TestingServoName, hardwareMap, AngleServoTestConfig.b_point1Position, AngleServoTestConfig.c_point1Angle, AngleServoTestConfig.d_point2Position, AngleServoTestConfig.e_point2Angle);
                } catch (Exception exception) {
                    telemetry.addLine("bad servo name prob" + exception);
                }
            }

            if (servo == null)
                return;

            servo.setPosition(targetPosition);

            telemetry.addLine("press a to refresh the servo");
            telemetry.addData("servo position", targetPosition);
            telemetry.addData("servo angle", servo.getAngle());


        }
    }
}
