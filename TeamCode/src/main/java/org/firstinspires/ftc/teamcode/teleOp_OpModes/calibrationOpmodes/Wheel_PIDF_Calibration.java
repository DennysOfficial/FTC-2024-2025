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

package org.firstinspires.ftc.teamcode.teleOp_OpModes.calibrationOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.individual_components.Pivot;
import org.firstinspires.ftc.teamcode.individual_components.VelocityControlDrive;
import org.firstinspires.ftc.teamcode.profiles_and_base_settings.Profiles;
import org.firstinspires.ftc.teamcode.profiles_and_base_settings.Settings;

@TeleOp(name = "Basic: Wheel PIDF Calibration", group = "Linear OpMode")
//@Disabled
public class Wheel_PIDF_Calibration extends LinearOpMode {




    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();

    private final ElapsedTime frameTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        Profiles driverProfiles = new Profiles(this); // creates an instance of the profiles class to access the date because I could figure out how to access it otherwise even though I think it should be possible
        Settings activeSettings = driverProfiles.defaultProfile; // selects the active setting that will be used in the rest of the code
        VelocityControlDrive velocityControlDrive = new VelocityControlDrive(this, activeSettings);

        waitForStart();
        runtime.reset();
        frameTimer.reset();

        double deltaTime = 0;

        double sensitivitySensitivity = 0.5;

        double sensitivity = 0.1;

        double[] PIDF_Values = new double[]{
                10, // p
                1, // i
                1, // d
                1  // f
        };

        String[] PIDF_ValueNames = new String[]{
                "P",
                "I",
                "D",
                "F"
        };

        int activeValue = 0;

        boolean previousButtonState = false;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            velocityControlDrive.updatePIDCoefficients();

            if (!previousButtonState && gamepad2.a)
                activeValue++;

            previousButtonState = gamepad2.a;

            if(activeValue >= 4)
                activeValue = 0;

            sensitivity *= 1 - gamepad2.right_stick_y * sensitivitySensitivity * deltaTime;

            PIDF_Values[activeValue] -= gamepad2.left_stick_y * sensitivity * deltaTime;

            deltaTime = frameTimer.seconds(); //gets the time since the start of last frame and then resets the timer
            frameTimer.reset();

            velocityControlDrive.updateWheels();


            // Show the elapsed game time and wheel power.
            telemetry.addData("Sensitivity = ", sensitivity);
            telemetry.addLine();
            telemetry.addData("Active value", "you are editing " + PIDF_ValueNames[activeValue] + " = " + PIDF_Values[activeValue]);
            telemetry.addLine();
            telemetry.addData("PIDF", "p %4.2f, i %4.2f, d %4.2f, f %4.2f",
                    velocityControlDrive.pidfCoefficients.p = PIDF_Values[0],
                    velocityControlDrive.pidfCoefficients.i = PIDF_Values[1],
                    velocityControlDrive.pidfCoefficients.d = PIDF_Values[2],
                    velocityControlDrive.pidfCoefficients.f = PIDF_Values[3]);
            telemetry.update();
        }
    }

}


