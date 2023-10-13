package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

@TeleOp(name="splineTestClaire", group="Example")
public final class splineTestCopy extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            /*MecanumDrive drive = new MecanumDrive(hardwareMap, (new Pose2d(60, -36, Math.toRadians(180))));
            */
            MecanumDrive drive = new MecanumDrive(hardwareMap, (new Pose2d(-36, -60, Math.toRadians(90))));
            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            // move to spike box
                            .setTangent(Math.toRadians(90))
                            .splineTo(new Vector2d(-36, -36), Math.toRadians(90))

                            //back out of spike box and maneuver for pixel stack
                            .setTangent(Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(-55, -48), Math.toRadians(180))


                            // approach stack for pickup
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(180)), Math.toRadians(180))


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

                            /* This is the code we originally had. The code not commented out is a copy and paste of rotated orientation
                            // move to spike box
                            .splineTo(new Vector2d(36, -36), Math.toRadians(180))
                            //back out of spike box and maneuver for pixel stack
                            .setTangent(Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(48, -55), Math.toRadians(180))
                            // approach stack for pickup
                            .setTangent(Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(12, -60, Math.toRadians(270)), Math.toRadians(270))
                            // drive across field toward backdrop
                            .setReversed(true)
                            .splineTo(new Vector2d(12, 24), Math.toRadians(90))
                            .setReversed(false)
                            //constant heading turn 2 to backdrop from drop off
                            .setTangent(Math.toRadians(90))
                            .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(90))

                             */


                    //spline test
                            /*
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(60, 0), Math.PI)

                             */

                            .build());

        } else{
            throw new AssertionError();
        }
    }
}
