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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.individual_components.Grabber;
import org.firstinspires.ftc.teamcode.individual_components.Intake;
import org.firstinspires.ftc.teamcode.individual_components.Lift;
import org.firstinspires.ftc.teamcode.individual_components.Pivot;
import org.firstinspires.ftc.teamcode.individual_components.VelocityControlDrive;
import org.firstinspires.ftc.teamcode.individual_components.Wrist;
import org.firstinspires.ftc.teamcode.misc.MotionControl;
import org.firstinspires.ftc.teamcode.profiles_and_base_settings.Profiles;
import org.firstinspires.ftc.teamcode.profiles_and_base_settings.Settings;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@TeleOp(name = "NATE-MODE", group = "Linear OpMode")
//@Disabled
public class NateManualControlOpMode extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime frameTimer = new ElapsedTime();




    @Override
    public void runOpMode() {


        Profiles driverProfiles = new Profiles(this); // creates an instance of the profiles class to access the date because I could figure out how to access it otherwise even though I think it should be possible
        Settings activeSettings = driverProfiles.defaultProfile; // selects the active setting that will be used in the rest of the code

        VelocityControlDrive activeDriveMode = new VelocityControlDrive(this, activeSettings);

        Grabber grabber = new Grabber(this, activeSettings);

        Lift lift = new Lift(this, activeSettings);
        lift.debugModeActive = true;

        Pivot pivot = new Pivot(this, activeSettings);
        pivot.debugModeEnabled = true;

        Wrist wrist = new Wrist(this, activeSettings);

        wrist.debugMode = true;

        Intake intake = new Intake(this, activeSettings);

        MotionControl motionControl = new MotionControl(pivot, lift, wrist, grabber, runtime, this, activeSettings);

        wrist.wristServo2.setAngle(0);
        pivot.setTargetPositionDeg(289);
        intake.setIntakeLift(.22);

        telemetry.addData("active grabber", "%s", activeSettings.activeGrabber);
        telemetry.update();

        waitForStart();
        runtime.reset();
        frameTimer.reset();

        double deltaTime = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            deltaTime = frameTimer.seconds(); //gets the time since the start of last frame and then resets the timer
            telemetry.addData("deltaTime ", deltaTime);
            frameTimer.reset();

            motionControl.updateMotionControl();

            activeDriveMode.updateWheels();

            grabber.directControl();



            //wrist.servo1directControl(deltaTime);
            motionControl.hingeWristAuto(deltaTime);
            //wrist.servo2directControl(deltaTime);
            //wrist.wristServo2.setAngle(30 * activeSettings.getWrist2Stick());

            intake.updateIntake(deltaTime);

            lift.runLiftPID(deltaTime);

            if (!motionControl.liftBusy()) {
                lift.directControl(deltaTime);
            }

            if (!motionControl.pivotBusy()) {
                pivot.directControl(deltaTime);
            }


            if (activeSettings.getButtonMoveToIntake()) {
                //lift.setPositionInch(14.98);
                motionControl.smoothMoveLift(0.31, 4);
                //pivot.setTargetPositionDeg(105.4);
                motionControl.smoothMovePivot(329.33, 4);
                wrist.wristServo2.setAngle(-0.05);
                motionControl.pose.targetGrabberAngle = 383.1;
                intake.setIntakeLift(0.58);
                grabber.setGrabber1Grabbing(true);
                grabber.setGrabber2Grabbing(true);
            }

            if (activeSettings.getPosPreBoard()) {
                //lift.setPositionInch(14.98);
                motionControl.smoothMoveLift(6.88, 2);
                //pivot.setTargetPositionDeg(105.4);
                motionControl.smoothMovePivot(121.71, 2);
                //wrist.wristServo1.setAngle(6.7);
                wrist.wristServo2.setAngle(0);
                motionControl.pose.targetGrabberAngle = 0;
            }

            if (gamepad2.dpad_up) {
                //lift.setPositionInch(14.98);
                motionControl.smoothMoveLift(0, 2);
                //pivot.setTargetPositionDeg(105.4);
                motionControl.smoothMovePivot(310, 2);
                //wrist.wristServo1.setAngle(6.7);
                wrist.wristServo2.setAngle(0);
                motionControl.pose.targetGrabberAngle = 100;
            }



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
