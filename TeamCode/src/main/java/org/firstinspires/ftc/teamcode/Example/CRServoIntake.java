package org.firstinspires.ftc.teamcode.Example;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp(name = "Example: CRServo Intake", group = "Example")
//@Disabled
public class CRServoIntake extends LinearOpMode
{
    CRServo leftIntake;
    CRServo rightIntake;
    @Override
    public void runOpMode() {
    leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
    rightIntake = hardwareMap.get(CRServo.class, "rightIntake");
    telemetry.addData(">", "Press Start to move Servo." );
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){

            // Set the contServo direction and power; // Intaking
            leftIntake.setDirection(DcMotorSimple.Direction.REVERSE );
            leftIntake.setPower(1.0);
            rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
            rightIntake.setPower(1.0);
            //idle();
        }
        // Display the current value
        telemetry.addData(">", "Press Stop to end test." );
        telemetry.update();

    }

}
