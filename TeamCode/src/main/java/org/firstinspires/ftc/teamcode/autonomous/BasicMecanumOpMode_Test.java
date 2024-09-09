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

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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


@TeleOp(name="Basic: Mecanum Linear OpMode Test", group="Linear OpMode")
//@Disabled
public class BasicMecanumOpMode_Test extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx liftMotorOne = null;
    private DcMotorEx liftMotorTwo = null;
    private DcMotorEx intake = null;
    private double liftPos = 0;

    private Servo leftup = null; //line from Jack. he he he haw
    private Servo rightup = null; //line from Jack. you would not believe your eyes if 10 million fireflies🔥🔥

    private Servo leftWrist = null; //derived from Jack. OOOHH EEE OOH I LOOK JUST LIKE BUDDY HOLLY ‼‼
    private Servo rightWrist = null; //derived from Jack. What about hamburger helper? I ate that food.
    private Servo leftGrab = null; //derived from Jack. water tastes better without the soap
    private Servo rightGrab = null; //derived from Jack. THE BLUETOOTH DEVICE IS READY TO PAIR!


    final double sensitivity = 0.01; //line from Jack. THERE IS NO "WISCONSIN"
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "1-FL");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "2-BL");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "3-BR");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "4-FR");
        liftMotorOne = hardwareMap.get(DcMotorEx.class, "1-Lift");
        liftMotorTwo = hardwareMap.get(DcMotorEx.class, "2-Lift");
        intake = hardwareMap.get(DcMotorEx.class, "chode");

        leftup = hardwareMap.get(Servo.class, "Pivot-0"); //line from Jack. feral hogs still in da yard 🔥🔥
        rightup = hardwareMap.get(Servo.class, "Pivot-1"); //line from Jack. yall watch judge judy?

        leftWrist = hardwareMap.get(Servo.class, "wrist 1"); //derived from Jack. WHEN YOU WIPEOUT ITS GONNA HURT 🔥🔥🔥
        rightWrist = hardwareMap.get(Servo.class, "wrist 2"); //derived from Jack. hello peter welcome to fortnite

        leftGrab = hardwareMap.get(Servo.class, "grab 1"); //derived from Jack. boots and cats and boots and cats
        rightGrab = hardwareMap.get(Servo.class, "grab 2"); //derived from Jack. the horses name was wednesday


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
//bro the comments are yapping 🗿

        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotorOne.setDirection(DcMotorEx.Direction.FORWARD);
        liftMotorOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotorTwo.setDirection(DcMotorEx.Direction.REVERSE);
        liftMotorTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //line from Jack.     where is super grover when you need him
        intake.setDirection(DcMotorEx.Direction.FORWARD); //line from Jack.        SIMPLE AND CLEAN IS THE WAAAAAAAY

        double currentPosition = 0; //line from Jack.        Everybody to the limit! Everybody to the limit! Everybody come on fhqwhgads!
        double otherCurrentPosistion = 0.0;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Lift One",liftMotorOne.getCurrentPosition());
        //telemetry.addData("Lift Two",liftMotorTwo.getCurrentPosition());
        telemetry.update();

        waitForStart();
        runtime.reset();
        //liftMotorTwo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        boolean previousButtonState = gamepad2.a;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double forwardBackward   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double strafe            =  gamepad1.left_stick_x;
            double yaw               =  gamepad1.right_stick_x;
            double liftUp            =  gamepad2.right_stick_y;

            currentPosition += gamepad2.left_stick_y*sensitivity; //line from Jack. did you hear they're ending young sheldon???
            currentPosition = Math.max(-.5, currentPosition); //line from Jack. have you ever played rugby? life is roblox. we the best music.
            currentPosition = Math.min(1,currentPosition);

            otherCurrentPosistion += -gamepad2.right_stick_x*sensitivity;
            otherCurrentPosistion = Math.max(-.5, otherCurrentPosistion);
            otherCurrentPosistion = Math.min(1,otherCurrentPosistion);



            if(gamepad1.right_bumper){
                intake.setPower(1.0); //line from Jack. Shoutout Brian Boitano! What would he do?
            }
            else intake.setPower(0.0); //line from Jack. nothing funny idk

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = forwardBackward + strafe*-1 + yaw;
            double rightFrontPower = forwardBackward + strafe*-1 - yaw;
            double leftBackPower   = forwardBackward - strafe*-1/1.5 + yaw;
            double rightBackPower  = forwardBackward - (strafe*-1)/1.5 - yaw;
            double liftOnePower    = liftUp;
            liftPos += liftUp * 0.5;


            liftMotorOne.setTargetPosition((int)liftPos);
            liftMotorOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //leftServo.setPosition(currentPosition+0.25873); line from Jack.READ ME✨ this line and next were from jacks, but commented out.
            //rightServo.setPosition(1-currentPosition);

            //liftMotorTwo.setTargetPosition((int)liftPos);


            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }



            boolean openClose = true;

            if (gamepad1.a && openClose && !previousButtonState){
                //run servo
                leftGrab.setPosition(.25); //probs wrong
                rightGrab.setPosition(.25); //maybe not true at all
                previousButtonState = gamepad1.a;
                openClose = false;

            }
            if (gamepad1.a && !openClose && !previousButtonState) {
                //run servo other way
                leftGrab.setPosition(-.25); //this is 100% not right
                rightGrab.setPosition(-.25); //dont trust this 💀💀
                previousButtonState = gamepad1.a;
                openClose = true;
            }

            previousButtonState = gamepad1.a;



            // Send calculated power to wheels
            leftFrontDrive.setVelocity(leftFrontPower*2000);
            rightFrontDrive.setVelocity(rightFrontPower*2000);
            leftBackDrive.setVelocity(leftBackPower*2000);
            rightBackDrive.setVelocity(rightBackPower*2000);
            liftMotorOne.setVelocity(liftOnePower*2000);
            liftMotorTwo.setVelocity(liftOnePower*2000);

            leftup.setPosition(currentPosition);
            rightup.setPosition(1-currentPosition);

            leftWrist.setPosition(otherCurrentPosistion+.25873);
            rightWrist.setPosition(currentPosition +.25873);




            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Test", "It Work");
            telemetry.addData("lifttttt", liftPos);
            telemetry.addData("left up", leftup.getPosition());
            telemetry.addData("right", rightup.getPosition());

            telemetry.addData("'a' button",previousButtonState); //line from Jack. buffalo buffalo buffalo buffalo buffalo buffalo buffalo buffalo buffalo
            telemetry.update();
        }
    }}
