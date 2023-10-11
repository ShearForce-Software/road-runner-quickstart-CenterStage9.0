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

public class Aidan_MeepMeepTesting_ExampleRoute2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17, 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-60, -36, Math.toRadians(0)))

                // https://rr.brott.dev/docs/v1-0/builder-ref/  Reference for trajectory segments

                // move to spike box
                .splineTo(new Vector2d(-36, -36), Math.toRadians(270))

                //backout and maneuver for pixel stack
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-48, -55), Math.toRadians(270))

                // drive forward toward the pixel stacks
                .setTangent(0)
                .splineTo(new Vector2d(-12, -55), Math.toRadians(0))

                // Approach to pixel stack
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-12, -60, Math.toRadians(270)), Math.toRadians(270))

                // Drive across field toward backdrop
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-12, 20, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-36, 48), Math.toRadians(180))
                // back to stack
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-12, 20), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-12, -60, Math.toRadians(270)), Math.toRadians(270))
               // to backboard and then to stack
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-12, 20, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-36, 48), Math.toRadians(180))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-12, 20), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-12, -60, Math.toRadians(270)), Math.toRadians(270))

                // manuever to drive under rigging back to backboard then back to stack
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-12, 20, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-36, 48), Math.toRadians(180))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-12, 20), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-12, -48, Math.toRadians(270)), Math.toRadians(270))

                /*
               // to new stack
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-24, -60, Math.toRadians(270)), Math.toRadians(90))

                // drive to backdrop
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-12, -50, Math.toRadians(270)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-12, 20, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-36, 48), Math.toRadians(180))

                // park in backstage
                .setTangent(270)
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-12, 55), Math.toRadians(90))
                */
                .build());


//        Image img = null;
//        try { img = ImageIO.read(new File("C:\\Users\\shear\\field-2023-juice-dark.png")); }
//        catch (IOException e) {}
//
//        meepMeep.setBackground(img)
//               .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
//                .start();



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();


    }
}