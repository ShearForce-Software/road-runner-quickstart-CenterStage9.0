package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.Action;

import java.util.concurrent.TimeUnit;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

import kotlin.reflect.KFunction;

public class Aidan_MeepMeepTesting_ExampleRoute3 {
    public static void main(String[] args) {
        int x = 0;
        int y = 0;

        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(55, 55, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17, 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-36, 60

                        , Math.toRadians(-90)))

                // https://rr.brott.dev/docs/v1-0/builder-ref/  Reference for trajectory segments

                // move to spike box
                //62.5 38.5
                .splineTo(new Vector2d(-36, 36), Math.toRadians(180))

                //backout and maneuver for pixel stack
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-50, 48), Math.toRadians(180))

                // drive forward toward the pixel stacks
                .setTangent(-90)
                .splineToConstantHeading(new Vector2d(-55, 12), Math.toRadians(-90))
                // Approach to pixel stack
                .setTangent(180)
                .splineToConstantHeading(new Vector2d(-60, 12), Math.toRadians(180))
                      //  .waitSeconds(10)

                // Drive across field toward backdrop

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(20, 12, Math.toRadians(180)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, 36), Math.toRadians(0))
                //commented until 1st meet ends
                // back to stack
             //   .setReversed(false)
               // .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(180))
             //   .splineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(180)), Math.toRadians(180))
                //commented until 1st meet ends
               // to backboard and then to stack
              //  .setReversed(true)
            //    .splineToLinearHeading(new Pose2d(20, 12, Math.toRadians(180)), Math.toRadians(0))
               // .splineToConstantHeading(new Vector2d(48, 36), Math.toRadians(0))
              //  .build());
       /* try
        {
            Thread.sleep(2000);
        }
        catch(InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
        */
                //commented until 1st meet ends
               // .setReversed(false)
                //.splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(180))
                //.splineToLinearHeading(new Pose2d(-60, 12, Math.toRadians(180)), Math.toRadians(180))

                // manuever to drive under rigging back to backboard then back to stack
                //commented until 1st meet ends
               // .setReversed(true)
                //.splineToLinearHeading(new Pose2d(20, 12, Math.toRadians(180)), Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(48, 36), Math.toRadians(0))
/*
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(20, 12), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-48, 12, Math.toRadians(180)), Math.toRadians(180))


               // to new stack
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-60, 24, Math.toRadians(180)), Math.toRadians(0))

                // drive to backdrop
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-50, 12, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(20, 12, Math.toRadians(180)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, 36), Math.toRadians(0))
*/
                // park in backstage
                .setTangent(90)
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(48, 12), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(63, 12, Math.toRadians(180)), Math.toRadians(0))

                        .build());


        Image img = null;
       try { img = ImageIO.read(new File("C:\\Users\\shear\\field-2023-juice-dark.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
               .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();


      /*  public void spikeMark()
        {
            //Use cameras and find the correct location for the team art object
            //Release team art onto the correct line
        }
        public void initialIntake()
        {
            //Activate the intake servos
            //Use sensors to make the intakes only take one pixel
        }
        static void raiseArm()
        {
            //Raise the arm by coding servos
            //Should occur around (12,12) on the Cartesian plane
        }
        static void lowerArm()
        {
         // Lower the arm by coding servos
         //Should occur around (12,12)
        }
        static void pixelDelivery()
        {
            //Code the robot to slightly decelerate to reduce the probability of collision
            // Code the arm to drop the pixel on back board.
        }
        static void generalIntake()
        {
            // Code the sensors to stop moving after the robot has two pixels
        }
      /*  meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
        */

    }
}