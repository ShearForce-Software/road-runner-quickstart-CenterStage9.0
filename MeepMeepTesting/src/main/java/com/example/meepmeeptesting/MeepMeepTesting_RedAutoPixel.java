package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_RedAutoPixel {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17, 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, -60, Math.toRadians(90)))

                // https://rr.brott.dev/docs/v1-0/builder-ref/  Reference for trajectory segments

                //drive into spike box for pixel delivery
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(-36, -36), Math.toRadians(90))

                //back out of spike box and maneuver for pixel stack approach
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-58, -48, Math.toRadians(180)), Math.toRadians(180))

                // approach stack for pickup
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-60, -11, Math.toRadians(180)), Math.toRadians(180))

                // drive across field toward backdrop
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(24, -11), Math.toRadians(0))

                // approach backdrop and drop off pixel (yellow pixel & 1 white)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -36), Math.toRadians(0))

                // return to center field
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(24, -11), Math.toRadians(180))

                // drive to pixel stack for pickup #2
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-60, -11), Math.toRadians(180))

                // drive across field toward backdrop
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(24, -11), Math.toRadians(0))

                // approach backdrop and drop off pixel (2 & 3 white)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -36), Math.toRadians(0))

                // return to center field
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(24, -11), Math.toRadians(180))

                // drive to pixel stack for pickup #3
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-60, -11), Math.toRadians(180))

                // drive across field toward backdrop
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(24, -11), Math.toRadians(0))

                // approach backdrop and drop off pixel (4 & 5 white)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -36), Math.toRadians(0))

                // return to center field
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(24, -11), Math.toRadians(180))

                // drive to location to maneuver to pixel stack 2
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-48, -11), Math.toRadians(180))

                // drive to location to maneuver to pixel stack 2
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-60, -24), Math.toRadians(180))

                // drive to pixel stack for pickup #3
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-48, -11), Math.toRadians(0))

                // cycle four

                // drive across field toward backdrop
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(24, -11), Math.toRadians(0))

                // approach backdrop to drop off pixel
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -36), Math.toRadians(0))

                // turn to park in backstage
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(55, -11), Math.toRadians(0))

//                //drive to center of spike box
//                .splineTo(new Vector2d(-36, -36), Math.toRadians(90))
//
//                //back out of spike box
//                .setTangent(Math.toRadians(270))
//                .splineTo(new Vector2d(-36, -41), Math.toRadians(270))
//
//                // rotate intake and position for initial approach to pixel stack
//                .splineTo(new Vector2d(-50, -48), Math.toRadians(180))
//
//                // drive to far pixel stack for single pixel pickup
//                .splineToConstantHeading(new Vector2d(-60, -12), Math.toRadians(180))
//
//                // cross straight across field under stage door
//                .setTangent(Math.toRadians(90))
//                .splineTo(new Vector2d(12, 24), Math.toRadians(90))
//
//                // drive to backdrop for pixel delivery
//                .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(90))
//                .setReversed(true)
//
//                // drive to position in prep for drive across field to pixel stack
//                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
//
//                // drive to pixel stack pickup for two pixels
//                .splineToConstantHeading(new Vector2d(12, -60), Math.toRadians(270))
//
//                // cross straight across field under stage door
//                .setTangent(Math.toRadians(90))
//                .splineTo(new Vector2d(12, 24), Math.toRadians(90))
//
//                // drive to backdrop for pixel delivery
//                .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(90))
//                .setReversed(true)
//
//                // drive to position in prep for drive across field to pixel stack
//                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
//
//                // drive to pixel stack pickup for two pixels
//                .splineToConstantHeading(new Vector2d(12, -60), Math.toRadians(270))
//
//                // cross straight across field under stage door
//                .setTangent(Math.toRadians(90))
//                .splineTo(new Vector2d(12, 24), Math.toRadians(90))
//
//                // drive to backdrop for pixel delivery
//                .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(90))
//                .setReversed(true)
//
//                // drive to position in prep for drive across field to pixel stack
//                .splineToConstantHeading(new Vector2d(12, 24), Math.toRadians(270))
//
//                // drive to position for 2nd stack pixel stack pickup
//                .splineToConstantHeading(new Vector2d(12, -48), Math.toRadians(270))
//
//                // drive to 2nd pixel stack two pixel pickup
//                .splineToConstantHeading(new Vector2d(24, -60), Math.toRadians(270))
//
//                // drive to position in prep for drive across field to pixel stack
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(12, -48), Math.toRadians(90))
//
//                // cross straight across field under stage door
//                .setTangent(Math.toRadians(90))
//                .splineTo(new Vector2d(12, 24), Math.toRadians(90))
//                // drive to backdrop for pixel delviery
//                .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(90))


                //.splineTo(new Vector2d(12, -61), Math.toRadians(-90))
                //.lineToX(36)
//                .turn(Math.toRadians(-90))
//                .lineToY(12)
//                .turn(Math.toRadians(-90))
//                .lineToX(60)
//                .turn(Math.toRadians(-90))
//                .lineToY(-36)
//                .turn(Math.toRadians(-90))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}