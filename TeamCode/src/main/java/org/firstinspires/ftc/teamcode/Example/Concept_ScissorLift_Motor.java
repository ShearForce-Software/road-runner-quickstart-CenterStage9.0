package org.firstinspires.ftc.teamcode.Example;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This OpMode tests HexCore motor for Gertude's scissor lift.
 *
 * This code assumes a motors configured with the following names:
 * DcMotor leftScissor;  Port 3
 * DcMotor rightFrScissor;

 */
@TeleOp(name = "Example: Gertrude Left Scissor Motor", group = "Example")
//@Disabled
public class Concept_ScissorLift_Motor extends LinearOpMode {

    // Define class members
    DcMotor leftScissor;

    @Override
    public void runOpMode() {

        // Hardware map

        leftScissor = hardwareMap.get(DcMotor.class, "leftScissor");


        // Initialize servo

        leftScissor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Telemetry
        telemetry.addData(">", "Press Start to test.");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {

            // Check gamepad for activation

            // power scissor motor
            if (gamepad1.left_trigger != 0) {
                leftScissor.setPower(gamepad1.left_trigger);
            } else if (gamepad1.right_trigger != 0) {
                leftScissor.setPower(-gamepad1.right_trigger);
            } else {
                leftScissor.setPower(0);


            }


            // Display the current value
            telemetry.addData("Left Scissor Motor Power", "%5.2f", leftScissor.getPower());
            telemetry.addData(">", "Press Stop to end test.");

            telemetry.update();
        }
    }}



