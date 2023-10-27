package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting_RedAuto_SplineOnly_Sandbox {
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

                //rotate in spike box for dropoff
                .splineTo(new Vector2d(-35, -38), Math.toRadians(-90))

                //move out of spike box and maneuver for pixel stack approach
                .splineTo(new Vector2d(-52, -48), Math.toRadians(180))


                // approach stack for pickup
                .splineTo(new Vector2d(-60, -11), Math.toRadians(180))

                // drive across field toward backdrop
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(24, -11), Math.toRadians(0))

                // approach backdrop and drop off pixel (yellow pixel & 1 white)
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(48, -36), Math.toRadians(0))

                // return to center field
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(24, -11), Math.toRadians(180))

                // drive to pixel stack for pickup #2
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(-60, -11), Math.toRadians(180))

                // drive across field toward backdrop
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(24, -11), Math.toRadians(0))

                // approach backdrop and drop off pixel (2 & 3 white)
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(48, -36), Math.toRadians(0))

                // return to center field
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(24, -11), Math.toRadians(180))

                // drive to pixel stack for pickup #3
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(-60, -11), Math.toRadians(180))

                // drive across field toward backdrop
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(24, -11), Math.toRadians(0))

                // approach backdrop and drop off pixel (4 & 5 white)
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(48, -36), Math.toRadians(0))

                // return to center field
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(24, -11), Math.toRadians(180))

                // drive to location to maneuver to pixel stack 2
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(-48, -11), Math.toRadians(180))

                // drive to pixel stack for pickup #4
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(-60, -24), Math.toRadians(180))

                // drive to position in prep for drive across field to backdrop
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(-48, -11), Math.toRadians(0))


                // drive across field toward backdrop
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(24, -11), Math.toRadians(0))

                // approach backdrop and drop off pixel (6 & 7 white)
                .setTangent(Math.toRadians(0))
                .splineTo(new Vector2d(48, -36), Math.toRadians(0))

                // turn to park in backstage
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(55, -11), Math.toRadians(0))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}