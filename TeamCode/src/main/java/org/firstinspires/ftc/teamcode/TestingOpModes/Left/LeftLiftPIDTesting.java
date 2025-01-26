package org.firstinspires.ftc.teamcode.TestingOpModes.Left;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftLift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.LeftPivot;


@TeleOp(name = "Left Lift PID", group = "Linear OpMode")
//@Disabled
public class LeftLiftPIDTesting extends LinearOpMode {


    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime frameTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); // if multiple sensors read calls are made in a row it will group them together making code more fasterer
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk

        RobotConfig activeConfig = new RobotConfig(this); // selects the active setting that will be used in the rest of the code


        LeftLift leftLift = new LeftLift(ControlAxis.ControlMode.gamePadVelocityControl,this, activeConfig);

        LeftPivot spinnyBit = new LeftPivot(ControlAxis.ControlMode.gamePadTorqueControl,this, activeConfig);

        spinnyBit.assignLift(leftLift);
        leftLift.assignPivot(spinnyBit);


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

            leftLift.update();

            spinnyBit.update();


            telemetry.addData("Run Time: ", runtime.toString());
            telemetry.update();
        }
    }

}
