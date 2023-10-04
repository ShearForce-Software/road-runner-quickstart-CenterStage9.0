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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -36, Math.toRadians(180)))

                // https://rr.brott.dev/docs/v1-0/builder-ref/  Reference for trajectory segments

                .lineToX(36)
                .turn(Math.toRadians(-90))
                .lineToY(12)
                .turn(Math.toRadians(-90))
                .lineToX(60)
                .turn(Math.toRadians(-90))
                .lineToY(-36)
                .turn(Math.toRadians(-90))
                .build());


       /* Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\chris\\OneDrive\\Desktop\\FTC\\4_Center Stage\\field-2023-official.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
            */
       meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}