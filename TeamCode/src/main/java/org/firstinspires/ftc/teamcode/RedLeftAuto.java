package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Left")
public class RedLeftAuto extends LinearOpMode {
    UniversalControlClass control = new UniversalControlClass(true, false,this);
    private HuskyLens huskyLens;
    private final int READ_PERIOD = 1;
    int leftRightSpikeBound = 150;
    int autoPosition = 0;
    public double pixelDeliverFirstPos = 14.5;
    public void runOpMode(){
        Pose2d startPose = new Pose2d(-38.5,-62.5,Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        control.Init(hardwareMap);;
        control.AutoStartPos();

        while(!isStarted()){
            control.HuskyLensInit();
            control.DetectTeamArtRed();
            telemetry.update();
        }

        control.WebcamInit(hardwareMap);
        telemetry.update();

        waitForStart();
        control.GrabPixels();

        Actions.runBlocking(
            drive.actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -14.5, Math.toRadians(90)), Math.toRadians(90))
                .build());
        control.DropOnLine();
        control.SafeStow();
    Actions.runBlocking(
            drive.actionBuilder(new Pose2d(-38.5, -14.5, Math.toRadians(90)))
                    .splineToLinearHeading(new Pose2d(-38,-10, Math.toRadians(90)), Math.toRadians(90))
                    .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-30,-10, Math.toRadians(180)), Math.toRadians(0))
            //.setTangent(0)
                    .splineToLinearHeading(new Pose2d(30,-10, Math.toRadians(180)), Math.toRadians(0))
                    .splineToLinearHeading(new Pose2d(48,-36, Math.toRadians(180)), Math.toRadians(0))
            .build());
        control.NavToTag();
//        Actions.runBlocking(
//                drive.actionBuilder(new Pose2d(48, -36, Math.toRadians(180)))
//                        .splineToLinearHeading(new Pose2d(48+control.rangeError,-36+control.yawError, Math.toRadians(180)), Math.toRadians(180))
//                        .build());
    }
}