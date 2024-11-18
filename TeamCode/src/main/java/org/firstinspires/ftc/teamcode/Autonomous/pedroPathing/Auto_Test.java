package org.firstinspires.ftc.teamcode.Autonomous.pedroPathing;


import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.Autonomous.pedroPathing.pathGeneration.PathChain;

@Autonomous(name = "SoupcOpMode_PP_Small")
public class Auto_Test extends OpMode {

    //Points of Interest
    private final Pose startposeactual = new Pose(0+72, 63+72, Math.toRadians(270));  // This is where the robot starts
    private final Pose startpose = new Pose(0+72, 63+72, Math.toRadians(180));  // this is a POI that just so happens to be similar to the above pose. Make sure you use these two correctly
    private final Pose rungpose = new Pose(0+72, 36+72, Math.toRadians(270));
    private final Pose interrimpose = new Pose(-36 + 72, 36 + 72, Math.toRadians(180));
    private final Pose samplestartpose = new Pose(-36 + 72, 12 + 72, Math.toRadians(180));
    private final Pose sample1pose = new Pose(-48 + 72, 12 + 72, Math.toRadians(90));
    private final Pose sample2pose = new Pose(-60 + 72, 12 + 72, Math.toRadians(90));
    private final Pose observe1pose = new Pose(-48 + 72, 54 + 72, Math.toRadians(90));
    private final Pose observe2pose = new Pose(-60 + 72, 54 + 72, Math.toRadians(90));
    private final Pose pickuppose = new Pose(-24 + 72, 63 + 72, Math.toRadians(180));  // TODO: Make this more specific

    // List of paths the robot takes
    private Path toRungStart, toRung1, toRung2, toRung3, toPickup1, toPickup2;
    private PathChain moveSamples;

    // Other misc. stuff
    private Follower follower;




    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
