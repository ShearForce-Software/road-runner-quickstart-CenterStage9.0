package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Autonomous(name="Blue Right")
public class BlueRightAuto extends LinearOpMode {
    //UniversalControlClass control = new UniversalControlClass(true, false, this);
    private HuskyLens huskyLens;
    private final int READ_PERIOD = 1;
    int leftRightSpikeBound = 150;
    int autoPosition = 0;
    double pixelDeliverForward =
    public void runOpMode(){
        Pose2d startPose = new Pose2d(-38.5,62.5,-90);

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens1");
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
        telemetry.update();
        while(!isStarted()){
            HuskyLens.Block[] blocks = huskyLens.blocks();
            if (blocks.length > 0){
                int xVal = blocks[0].x;
                telemetry.addData("Team Art Detected: ", true);
                telemetry.addData("Team Art X position: ", xVal);
                //x value ranges from left to right 0 to 320, with 160 being the center
                //create variables for x min/max values for each spike mark location 1,2,3
                //determine position and assign variable for drive in autonomous
                if (xVal < leftRightSpikeBound){
                    autoPosition = 2;
                }
                else if (xVal > leftRightSpikeBound){
                    autoPosition = 3;
                }
                telemetry.addData("Auto position: ", autoPosition);
            }
            else{
                //pick a spot
                telemetry.addData("!!Team Art NOT DETECTED!! ", "DEFAULT TO CENTER");
                autoPosition = 2;
            }
            telemetry.update();
        }
        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(drive.pose)
                .lineToX(24)
                .build());
    }
}
