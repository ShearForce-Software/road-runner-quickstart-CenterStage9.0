package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting_ExampleRoute1 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17, 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -36, Math.toRadians(180)))

                // https://rr.brott.dev/docs/v1-0/builder-ref/  Reference for trajectory segments

                // move to spike box
                .splineTo(new Vector2d(36, -36), Math.toRadians(180))

                //back out of spike box and maneuver for pixel stack
//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(55, -48, Math.toRadians(270)), Math.toRadians(270))

                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -55), Math.toRadians(180))

                // drive forward toward the pixel stacks
                //.splineTo(new Vector2d(15, -55), Math.toRadians(180))

                // approach stack for pickup
//                .setTangent(0)
//                .splineToConstantHeading(new Vector2d(12, -60), Math.toRadians(270))

                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(12, -60, Math.toRadians(270)), Math.toRadians(270))


                //.splineToSplineHeading(new Pose2d(12, -60, Math.toRadians(270)), Math.toRadians(270))

                // drive across field toward backdrop
                .setReversed(true)
                .splineTo(new Vector2d(12, 24), Math.toRadians(90))
                .setReversed(false)
//                //constant heading turn 1 to backdrop
//                .setTangent(90)
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(24, 36), Math.toRadians(0))
//
//                //constant heading turn 2 to backdrop from drop off
//                .setTangent(0)
//                .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(90))

                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(90))
                // manuever to drive under rigging back to pixel stack
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))

                // drive to pixel stack for pickup #2
                .splineTo(new Vector2d(12, -60), Math.toRadians(270))

                // drive across field toward backdrop
                .setReversed(true)
                .splineTo(new Vector2d(12, 24), Math.toRadians(90))

                //constant heading turn 1 to backdrop
                .setTangent(90)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(24, 36), Math.toRadians(0))

                //constant heading turn 2 to back drop from drop off
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(90))

                // manuever to drive under rigging back to pixel stack
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))

                // drive to pixel stack for pickup #3
                .splineTo(new Vector2d(12, -60), Math.toRadians(270))

                // drive across field toward backdrop
                .setReversed(true)
                .splineTo(new Vector2d(12, 24), Math.toRadians(90))

                //constant heading turn 1 to backdrop
                .setTangent(90)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(24, 36), Math.toRadians(0))

                //constant heading turn 2 to back drop from drop off
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(90))

                // turn to park in backstage
                .setTangent(270)
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(12, 55), Math.toRadians(90))

                .build());


        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\chris\\OneDrive\\Desktop\\FTC\\4_Center Stage\\field-2023-official.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();

//        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
//                .start();
    }
}