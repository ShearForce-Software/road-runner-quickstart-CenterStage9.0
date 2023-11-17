package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Test Auto")
public class TestAutonomous extends LinearOpMode {
    //UniversalControlClass control = new UniversalControlClass(true, false, this);
    public void runOpMode(){
        Pose2d startPose = new Pose2d(0,0,0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.update();
        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .splineToConstantHeading(new Vector2d(0, 36), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(36, 36), Math.toRadians(0))
                .setReversed(true)
                .splineToSplineHeading(startPose, Math.toRadians(-90))
                .build());
    }
}
