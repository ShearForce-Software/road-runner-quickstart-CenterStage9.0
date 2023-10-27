package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_RedAutoBackDrop {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17, 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, -60, Math.toRadians(90)))

                // https://rr.brott.dev/docs/v1-0/builder-ref/  Reference for trajectory segments

                // move into spike box for drop off  TODO: Detect teamart, position for drop off, position for backdrop delivery
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(12, -36), Math.toRadians(90))

                //back out of spike box and maneuver for backdrop
                .setTangent(Math.toRadians(-90))
                //drive to X, Y coordinate, Face 180, Move 0
                .splineToLinearHeading(new Pose2d(24, -48, Math.toRadians(180)), Math.toRadians(0))

                // approach backdrop and drop off pixel # 1
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -36), Math.toRadians(0))

                // approach wall to position for pixel run
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(0, -58), Math.toRadians(180))

                // drive under rigging toward near pixel stack
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-48, -58), Math.toRadians(180))

                // approach near pixel stack for pickup
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-60, -35), Math.toRadians(180))

                // position to drive under rigging toward backdrop
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-48, -58), Math.toRadians(0))

                // drive under rigging toward toward backdrop
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(0, -58), Math.toRadians(0))

                // approach backdrop and drop off pixel # 2 & 3
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -36), Math.toRadians(0))

                // next cycle

                // approach wall to position for pixel run
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(0, -58), Math.toRadians(180))

                // drive under rigging toward near pixel stack
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-48, -58), Math.toRadians(180))

                // approach near pixel stack for pickup
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-60, -35), Math.toRadians(180))

                // position to drive under rigging toward backdrop
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-48, -58), Math.toRadians(0))

                // drive under rigging toward toward backdrop
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(0, -58), Math.toRadians(0))

                // approach backdrop and drop off pixel # 4 & 5
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -36), Math.toRadians(0))

                // next cycle

                // approach wall to position for pixel run
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(0, -58), Math.toRadians(180))

                // drive under rigging toward near pixel stack
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-48, -58), Math.toRadians(180))

                // approach near pixel stack for pickup
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-60, -35), Math.toRadians(180))

                // position to drive under rigging toward backdrop
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-48, -58), Math.toRadians(0))

                // drive under rigging toward toward backdrop
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(0, -58), Math.toRadians(0))

                // approach backdrop and drop off pixel # 6 & 7
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -36), Math.toRadians(0))

                // turn to park in backstage
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(55, -58), Math.toRadians(0))


                // position to wall to prep for pixel run
                //.setTangent(Math.toRadians(-90))
                //.splineToLinearHeading(new Pose2d(24, -60, Math.toRadians(180)), Math.toRadians(180))

//                // approach backdrop and drop off pixel
//                .setTangent(Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(48, -60), Math.toRadians(180))
//
//                // drive across field toward backdrop
//                .setReversed(true)
//                .splineTo(new Vector2d(24, -60), Math.toRadians(0))
//                .setReversed(false)
//
//                // approach backdrop and drop off pixel
//                .setTangent(Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(48, -36), Math.toRadians(0))
//
//                // return to center field
//                .setTangent(90)
//                .splineToConstantHeading(new Vector2d(24, -12), Math.toRadians(180))
//
//                // drive to pixel stack for pickup #2
//                .splineTo(new Vector2d(-60, -12), Math.toRadians(180))
//
//                // drive across field toward backdrop
//                .setReversed(true)
//                .splineTo(new Vector2d(24, -12), Math.toRadians(0))
//
//
//                // approach backdrop and drop off pixel
//                .setTangent(0)
//                .splineToConstantHeading(new Vector2d(48, -36), Math.toRadians(0))
//
//                // return to center field
//                .setTangent(90)
//                .splineToConstantHeading(new Vector2d(24, -12), Math.toRadians(180))
//
//                // drive to pixel stack for pickup #3
//                .splineTo(new Vector2d(-60, -12), Math.toRadians(180))
//
//                // drive across field toward backdrop
//                .setReversed(true)
//                .splineTo(new Vector2d(24, -12), Math.toRadians(0))
//
//
//                // approach backdrop to drop off pixel
//                .setTangent(0)
//                .splineToConstantHeading(new Vector2d(48, -36), Math.toRadians(0))
//
//                // turn to park in backstage
//                .setTangent(180)
//                .setReversed(false)
//                .splineToConstantHeading(new Vector2d(55, -12), Math.toRadians(0))

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