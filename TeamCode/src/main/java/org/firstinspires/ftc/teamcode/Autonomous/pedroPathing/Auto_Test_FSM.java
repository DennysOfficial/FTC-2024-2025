package org.firstinspires.ftc.teamcode.Autonomous.pedroPathing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.Config.RobotConfig;
import org.firstinspires.ftc.teamcode.individual_components.ControlAxis;
import org.firstinspires.ftc.teamcode.individual_components.Pivot;
import org.firstinspires.ftc.teamcode.individual_components.grabbers.ActiveIntake;

import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.Point;

import static java.lang.Thread.sleep;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */
@Disabled
@Autonomous(name = "SoupcOpMode_PP_Small_FSM")
public class Auto_Test_FSM extends LinearOpMode {

    //Points of Interest
    private final Pose startposeactual =  new Pose( 00 + 72,  63 + 72, Math.toRadians(270));  // This is where the robot starts
    private final Pose startpose =        new Pose( 00 + 72,  63 + 72, Math.toRadians(180));  // this is a POI that just so happens to be similar to the above pose. Make sure you use these two correctly
    private final Pose rungpose =         new Pose( 00 + 72,  36 + 72, Math.toRadians(270));
    private final Pose rungpose1 =        new Pose(-03 + 72,  36 + 72, Math.toRadians(270));
    private final Pose rungpose2 =        new Pose( 03 + 72,  36 + 72, Math.toRadians(270));
    private final Pose rungpose3 =        new Pose( 06 + 72,  36 + 72, Math.toRadians(270));
    private final Pose rungposecurve =    new Pose(-06 + 72,  48 + 72, Math.toRadians(270));
    private final Pose interrimpose =     new Pose(-36 + 72,  36 + 72, Math.toRadians(180));
    private final Pose samplestartpose =  new Pose(-36 + 72,  12 + 72, Math.toRadians(180));
    private final Pose sample1pose =      new Pose(-48 + 72,  12 + 72, Math.toRadians(90));
    private final Pose sample2pose =      new Pose(-60 + 72,  12 + 72, Math.toRadians(90));
    private final Pose observe1pose =     new Pose(-48 + 72,  54 + 72, Math.toRadians(90));
    private final Pose observe2pose =     new Pose(-60 + 72,  54 + 72, Math.toRadians(90));
    private final Pose pickuppose =       new Pose(-24 + 72,  63 + 72, Math.toRadians(180));  // TODO: Make this more specific

    // List of paths the robot takes
    private Path toRungStart, toRung1, toRung2, toRung3, toPickup1, toPickup2;
    private PathChain moveSamples, scoreSpecimens;

    // Other misc. stuff
    private Follower follower;

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        TO_RUNG_START,
        TO_RUNG1,
        TO_RUNG2,
        TO_RUNG3,
        TO_PICKUP1,
        TO_PICKUP2,
        MOVE_SAMPLES,
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime frameTimer = new ElapsedTime();

    double deltaTime;

    Trajectory toRung;
    Trajectory toWall;
    Trajectory toNetZone;

    Pose2d startPose = new Pose2d(new Vector2d(60, 0), Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our lift
        RobotConfig activeConfig = new RobotConfig(this);

        org.firstinspires.ftc.teamcode.individual_components.Lift lift = new org.firstinspires.ftc.teamcode.individual_components.Lift(this, activeConfig);

        lift.setControlMode(ControlAxis.ControlMode.directControl);

        Pivot spinyBit = new Pivot(this, activeConfig);

        spinyBit.setControlMode(ControlAxis.ControlMode.directControl);

        //Pincher pincher = new Pincher(this,activeConfig);

        ActiveIntake intake = new ActiveIntake(this, activeConfig);


        RobotConfig config = new RobotConfig(this);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        toRungStart = new Path(new BezierLine(new Point(startposeactual), new Point(rungpose)));
        toRung1 =   new Path(new BezierCurve(new Point(pickuppose), new Point(rungposecurve), new Point(rungpose1)));
        toRung2 =   new Path(new BezierCurve(new Point(pickuppose), new Point(rungposecurve), new Point(rungpose2)));
        toRung3 =   new Path(new BezierCurve(new Point(pickuppose), new Point(rungposecurve), new Point(rungpose3)));
        toPickup1 = new Path(new BezierCurve(new Point(rungpose1),  new Point(rungposecurve), new Point(pickuppose)));
        toPickup2 = new Path(new BezierCurve(new Point(rungpose2),  new Point(rungposecurve), new Point(pickuppose)));

        moveSamples = follower.pathBuilder()
                .addPath(new BezierLine  (new Point(rungpose),        new Point(interrimpose)))
                .addPath(new BezierLine  (new Point(interrimpose),    new Point(samplestartpose)))
                .addPath(new BezierLine  (new Point(samplestartpose), new Point(sample1pose)))
                .addPath(new BezierLine  (new Point(sample1pose),     new Point(observe1pose)))
                .addPath(new BezierLine  (new Point(observe1pose),    new Point(sample1pose)))
                .addPath(new BezierLine  (new Point(sample1pose),     new Point(sample2pose)))
                .addPath(new BezierLine  (new Point(sample2pose),     new Point(observe2pose)))
                .addPath(new BezierCurve (new Point(observe2pose),    new Point(observe1pose), new Point(pickuppose)))
                .build();

        scoreSpecimens = follower.pathBuilder()
                .addPath(toRung1)
                .addPath(toPickup1)
                .addPath(toRung2)
                .addPath(toPickup2)
                .addPath(toRung3)
                .build();



        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        deltaTime = 0;
        frameTimer.reset();
        currentState = State.TO_RUNG_START;
        follower.followPath(toRungStart);


        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case TO_RUNG_START:
                    telemetry.addLine("It work 1");
                    telemetry.update();
                    // Check if the drive class is busy turning
                    // If not, move onto the next state
                    // We are done with the program
                    if (!drive.isBusy()) {
                        currentState = State.MOVE_SAMPLES;
                        follower.followPath(moveSamples);
                    }
                    break;
                case MOVE_SAMPLES:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state
                    // We are done with the program
                    if (!drive.isBusy()) {
                        Thread.sleep(0);
                        currentState = State.TO_RUNG1;
                        follower.followPath(moveSamples);
                    }
                case TO_RUNG1:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state
                    // We are done with the program
                    if (!drive.isBusy()) {
                        Thread.sleep(0);
                        currentState = State.TO_PICKUP1;
                        follower.followPath(toPickup1);
                    }
                case TO_PICKUP1:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state
                    // We are done with the program
                    if (!drive.isBusy()) {
                        Thread.sleep(0);
                        currentState = State.TO_RUNG2;
                        follower.followPath(toRung2);
                    }
                case TO_RUNG2:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state
                    // We are done with the program
                    if (!drive.isBusy()) {
                        Thread.sleep(0);
                        currentState = State.TO_PICKUP2;
                        follower.followPath(toPickup2);
                    }
                case TO_PICKUP2:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state
                    // We are done with the program
                    if (!drive.isBusy()) {
                        Thread.sleep(0);
                        currentState = State.TO_RUNG3;
                        follower.followPath(toRung3);
                    }
                case TO_RUNG3:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state
                    // We are done with the program
                    if (!drive.isBusy()) {
                        Thread.sleep(0);
                        currentState = State.IDLE;
                    }
                case IDLE:
                    telemetry.addLine("It work 2");
                    telemetry.update();
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            deltaTime = frameTimer.seconds(); //gets the time since the start of last frame and then resets the timer
            telemetry.addData("deltaTime", deltaTime);
            frameTimer.reset();

            drive.update();
            lift.update(deltaTime, spinyBit.getPosition());
            spinyBit.update(deltaTime, lift.getPosition());

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addLine("I am running");
            telemetry.update();
            // Put your PID Update Function Here
        }
    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
    class Lift {
        public Lift(HardwareMap hardwareMap) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware
        }

        public void update() {
            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift
        }
    }
}
