package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.individual_components.Lift;
import org.firstinspires.ftc.teamcode.individual_components.Pivot;
import org.firstinspires.ftc.teamcode.individual_components.grabbers.ActiveIntake;

@Config
@Autonomous(name = "soupcOpMode_Simple_Specimen")
public class Automous_Async_Simple_2 extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime frameTimer = new ElapsedTime();

    Trajectory toRung;
    Pose2d startPose = new Pose2d(new Vector2d(60, 0), Math.toRadians(180));

    @Override
    public void runOpMode() {

        RobotConfig activeConfig = new RobotConfig(this);

        Lift lift = new Lift(this, activeConfig);

        lift.setControlMode(ControlAxis.ControlMode.directControl);

        Pivot spinyBit = new Pivot(this, activeConfig);

        spinyBit.setControlMode(ControlAxis.ControlMode.directControl);

        //Pincher pincher = new Pincher(this,activeConfig);

        ActiveIntake intake = new ActiveIntake(this, activeConfig);


        RobotConfig config = new RobotConfig(this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this);

        drive.setPoseEstimate(startPose);

        toRung = drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(() -> {
                    spinyBit.setTargetPosition(0);
                    lift.setTargetPosition(8.8);
                })
                .splineTo(new Vector2d(0,0), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    //[place] SPECIMEN on RUNG
                    lift.setTargetPosition(0);
                })
                .build();

        while (!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectoryAsync(toRung);

        double deltaTime = 0;

        while (!isStopRequested() && opModeIsActive()) {

            deltaTime = frameTimer.seconds(); //gets the time since the start of last frame and then resets the timer
            telemetry.addData("deltaTime", deltaTime);
            frameTimer.reset();

            drive.update();
            lift.update(deltaTime, spinyBit.getPosition());
            spinyBit.update(deltaTime, lift.getPosition());

            // Put your PID Update Function Here
        }
    }
}