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
            MecanumDrive drive = new MecanumDrive(hardwareMap, (new Pose2d(60, -36, Math.toRadians(180))));

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
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
