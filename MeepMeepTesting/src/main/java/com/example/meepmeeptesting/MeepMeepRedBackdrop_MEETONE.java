package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRedBackdrop_MEETONE {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17, 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(14, -60, Math.toRadians(90)))


                // https://rr.brott.dev/docs/v1-0/builder-ref/  Reference for trajectory segments

                // move to spike box
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(14, -36), Math.toRadians(90))

                //Robot pause and either rotate and deliver pixel to one or three or drive forward

                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(14, -14), Math.toRadians(90))


                //Pause to deliver to spike mark 2
                //Rotate robot
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(14, -7), Math.toRadians(90))

                .setTangent(Math.toRadians(-90))
                .splineTo(new Vector2d(24, -14), Math.toRadians(0))

                //Pause here for several seconds for partner

                .setReversed(true)
                .splineTo(new Vector2d(38, -14), Math.toRadians(0))
                .setReversed(false)


                .splineToConstantHeading(new Vector2d(48, -36), Math.toRadians(0))

                //Navigate with Camera

                /*
                // drive across field toward backdrop
                .setReversed(true)
                .splineTo(new Vector2d(24, -12), Math.toRadians(0))
                .setReversed(false)

                // approach backdrop and drop off pixel
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -36), Math.toRadians(0))

                // return to center field
                .setTangent(90)
                .splineToConstantHeading(new Vector2d(24, -12), Math.toRadians(180))

                // drive to pixel stack for pickup #2
                .splineTo(new Vector2d(-60, -12), Math.toRadians(180))

                // drive across field toward backdrop
                .setReversed(true)
                .splineTo(new Vector2d(24, -12), Math.toRadians(0))


                // approach backdrop and drop off pixel
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(48, -36), Math.toRadians(0))

                // return to center field
                .setTangent(90)
                .splineToConstantHeading(new Vector2d(24, -12), Math.toRadians(180))

                // drive to pixel stack for pickup #3
                .splineTo(new Vector2d(-60, -12), Math.toRadians(180))

                // drive across field toward backdrop
                .setReversed(true)
                .splineTo(new Vector2d(24, -12), Math.toRadians(0))


                // approach backdrop to drop off pixel
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(48, -36), Math.toRadians(0))

                // turn to park in backstage
                .setTangent(180)
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(55, -12), Math.toRadians(0))
                */

                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();



    }

}