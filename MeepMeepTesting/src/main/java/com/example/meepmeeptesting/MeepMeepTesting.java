package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.net.URL;
import java.nio.file.Path;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(72-18f/2, 0, Math.toRadians(90)))
                                .forward(30)
                                .splineTo(new Vector2d(36, 0), Math.toRadians(180))
                                .strafeLeft(48.5)
                                .lineToLinearHeading(new Pose2d(54, -54, Math.toRadians(135)))
                                .splineTo(new Vector2d(12, -24), Math.toRadians(90))
                                .build()
                );

        Image img = null;
        try {
            img = ImageIO.read(new URL("https://preview.redd.it/into-the-deep-meepmeep-custom-field-images-printer-friendly-v0-qsax5fraignd1.png?width=1080&crop=smart&auto=webp&s=4bfa9fc15dc26c12019b35015d009446dcb724b9"));
        } catch (Exception e) {
        }


        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}