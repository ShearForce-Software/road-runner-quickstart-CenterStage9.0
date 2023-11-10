package org.firstinspires.ftc.teamcode;
import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp(name = "Test Blinkin", group = "Test")
//@Disabled
public class TestBlinkin extends LinearOpMode
{
    UniversalControlClass theRobot = new UniversalControlClass(true, true, this);;
    boolean button;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() {
        theRobot.Init(this.hardwareMap);
        //theRobot.SetLeftToColor(1, 1);
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive())

        {

            // Set the contServo direction and power; // Intaking
            // When a button is hit, select a color
            if (gamepad1.a == true)
            {
                theRobot.SetLeftToColor(2, 1);
            }
            else
            {
                theRobot.SetLeftToColor(3,1);
            }
            //idle();
        // Display the current value
        telemetry.addData(">", "Press Stop to end test." );
        telemetry.update();
        }

    }

}
