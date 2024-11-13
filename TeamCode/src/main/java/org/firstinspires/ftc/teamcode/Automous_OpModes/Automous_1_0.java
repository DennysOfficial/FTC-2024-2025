package org.firstinspires.ftc.teamcode.Automous_OpModes;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.AutonomouseStuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotStuff.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes.BasicMechanumDrive;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.DriveModes.DriveModeBase;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Lift;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.Pivot;
import org.firstinspires.ftc.teamcode.RobotStuff.individual_components.grabbers.ActiveIntake;
import org.firstinspires.ftc.teamcode.RobotStuff.stuffAndThings.Animator;

import java.util.List;

@Config
@Autonomous(name = "SoupcOpMode 1.0", group = "Autonomous")
public class Automous_1_0 extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime frameTimer = new ElapsedTime();

    RobotConfig activeConfig = new RobotConfig(this); // selects the active setting that will be used in the rest of the code


    DriveModeBase activeDriveMode = new BasicMechanumDrive(this, activeConfig);


    Lift lift = new Lift(this, activeConfig, runtime);



    Pivot spinnyBit = new Pivot(this, activeConfig, runtime);



    public class MoveLift implements Action {

        double position;
        public MoveLift(double Position) {
            this.position = Position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            lift.setTargetPosition(position);
            return false;
        }
    }
    public Action MoveLift(double position) {return new MoveLift(position);}


    public class MovePivot implements Action {

        double position;

        public MovePivot(double position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            spinnyBit.setControlMode(ControlAxis.ControlMode.positionControl);
            spinnyBit.setTargetPosition(position);
            return false;
        }
    }
    public Action MovePivot(double position) {return new MovePivot(position);}


    public class PivotUpdate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            spinnyBit.update();
            return false;
        }
    }
    public Action PivotUpdate() {return new PivotUpdate();}




    @Override
    public void runOpMode() {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }



        final ControlAxis.ControlMode defaultPivotControlMode = ControlAxis.ControlMode.positionControl;

        spinnyBit.setControlMode(defaultPivotControlMode);

        spinnyBit.assignLift(lift);
        lift.assignPivot(spinnyBit);

        final ControlAxis.ControlMode defaultLiftControlMode = ControlAxis.ControlMode.positionControl;

        lift.setControlMode(defaultLiftControlMode);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // does stuff for ftc dashboard idk



        //Pincher pincher = new Pincher(this,activeConfig);

        ActiveIntake intake = new ActiveIntake(this, activeConfig);


        Animator pivotControl = new Animator(runtime, this, activeConfig, spinnyBit, lift);


        Pose2d startPose = new Pose2d(63, 0, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);




        // vision here that outputs position


        /* toRung = drive.trajectoryBuilder(startPose)
                .addDisplacementMarker(() -> {
                    spinyBit.setTargetPosition(0);
                    lift.setTargetPosition(8.8);
                })
                .splineTo(new Vector2d(0,0), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    //[place] SPECIMEN on RUNG
                    lift.setTargetPosition(0);
                })
                .build(); */

        TrajectoryActionBuilder toRung = drive.actionBuilder(startPose)
                .afterDisp(0, MoveLift(8.8))
                .afterDisp(0, MovePivot(0))
                .lineToX(6)
                .afterDisp(0, MoveLift(0));



        /* TrajectoryActionBuilder tab1 = drive.actionBuilder(startPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(startPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(startPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.fresh()
                .strafeTo(new Vector2d(48, 12))
                .build(); */

        // actions that need to happen on init; for instance, a claw tightening.




        waitForStart();

        if (isStopRequested()) return;

        Action TORUNG = toRung.build();

        SequentialAction driveUpdate = new SequentialAction(
                TORUNG
        );

        ParallelAction routine = new ParallelAction(
                driveUpdate,
                Lift.LiftUpdate,
                Pivot.PivotUpdate
        );


        Actions.runBlocking(
                routine
        );
    }
}