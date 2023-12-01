package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17, 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-38.5,-62.5,Math.toRadians(90)))

                // https://rr.brott.dev/docs/v1-0/builder-ref/  Reference for trajectory segments
                .splineToLinearHeading(new Pose2d(-38.5, -14.5, Math.toRadians(90)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-38,-10, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-30,-10, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(30,-10, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48,-36, Math.toRadians(180)), Math.toRadians(0))
                .build());

       meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}