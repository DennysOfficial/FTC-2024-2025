package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.individual_components.Lift;
import org.firstinspires.ftc.teamcode.individual_components.Pivot;


@TeleOp(name = "Pivot Velocity Pid Test", group = "Linear OpMode")
@Config
@Disabled
public class PivotVelocityPIDTest extends LinearOpMode {


    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime frameTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); // if multiple sensors read calls are made in a row it will group them together making code more fasterer
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk


        RobotConfig activeConfig = new RobotConfig(this); // selects the active setting that will be used in the rest of the code


        Pivot spinyBit = new Pivot(this, activeConfig);

        spinyBit.setControlMode(ControlAxis.ControlMode.velocityControl);

        Lift lift = new Lift(this, activeConfig);

        lift.setControlMode(ControlAxis.ControlMode.directTorqueControl);


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

            if (gamepad2.dpad_down)
                spinyBit.setTargetVelocity(-20);
            else if (gamepad2.dpad_up)
                spinyBit.setTargetVelocity(20);
            else
                spinyBit.setTargetVelocity(activeConfig.inputMap.getPivotStick() * activeConfig.sensitivities.getPivotRate());


            spinyBit.update(deltaTime, lift.getPosition());
            lift.update(deltaTime, spinyBit.getPosition());


            telemetry.addData("Run Time: ", runtime.toString());
            telemetry.update();
        }
    }

}
