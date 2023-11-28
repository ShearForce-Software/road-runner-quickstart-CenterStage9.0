package org.firstinspires.ftc.teamcode.Example;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/*
 * This OpMode moves tests all of Gertude's servos and motors.
 *
 * This code assumes a motors configured with the following names:
 * DcMotor leftFront;
 * DcMotor leftRear;
 * DcMotor rightFront;
 * DcMotor rightRear;
 * DcMotor rightSlide;
 * DcMotor leftSlide;
 *
 * This code assumes a CRservos configured with the following names:
 * CRServo intakeLeft;
 * CRServo intakeRight;
 *
 * This code assumes a Servos configured with the following names:
 * Servo   pixelGrabberLeft;
 * Servo   pixelGrabberRight;
 * Servo   armRotateLeft;
 * Servo   armRotateRight;
 * Servo   pixelFlipLeft;
 * Servo   pixelFlipRight;
 *
 *
 *
 *
 */
@TeleOp(name = "Example: Gertrude Manual Control", group = "Example")
//@Disabled
public class Concept_Gertrude_ManualControl extends LinearOpMode {

    // Define class members
    CRServo intakeLeft;
    CRServo intakeRight;
    Servo   grabberServoLeft;
    Servo   grabberServoRight;
    Servo armRotLeft;
    Servo armRotRight;
    DcMotor slideLeft;
    DcMotor slideRight;
    TouchSensor limitSwitchLeft;
    TouchSensor limitSwitchRight;


    Servo pixelRotLeft;
    Servo pixelRotRight;

    static final double SCALE       = 0.001;     // Joystick scaling for servo increment value
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    static final double MIN_POS_ARM     =  0.04;     // Minimum rotational position

    double  pixelGrabberLeftPosition = 0;
    double  pixelGrabberRightPosition = 0;

    double armRotationLeftPosition = 0.07;
    double armRotationRightPosition = 0.07;
    double  pixelRotateLeftPosition = .7;
    double  pixelRotateRightPosition = .7;
    static final double MIN_POS_PIXEL     =  0.0;     // Minimum rotational position
    static final double MAX_POS_PIXEL     =  0.8;     // Minimum rotational position
    @Override
    public void runOpMode() {

        // Hardware map

        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        grabberServoLeft = hardwareMap.get(Servo.class, "pixelGrabberLeft");
        grabberServoRight = hardwareMap.get(Servo.class, "pixelGrabberRight");
        armRotLeft = hardwareMap.get(Servo.class, "armRotateLeft");
        armRotRight = hardwareMap.get(Servo.class, "armRotateRight");
        pixelRotLeft = hardwareMap.get(Servo.class, "pixelRotateLeft");
        pixelRotRight = hardwareMap.get(Servo.class, "pixelRotateRight");
        slideLeft = hardwareMap.get(DcMotor.class, "leftSlide");
        slideRight = hardwareMap.get(DcMotor.class, "rightSlide");
        limitSwitchLeft = hardwareMap.get(TouchSensor.class, "limitSwitchLeft");
        limitSwitchRight = hardwareMap.get(TouchSensor.class, "limitSwitchRight");
        // Initialize servo

        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
        grabberServoLeft.setDirection(Servo.Direction.REVERSE);
        grabberServoRight.setDirection(Servo.Direction.FORWARD);
        armRotLeft.setDirection(Servo.Direction.REVERSE);
        armRotRight.setDirection(Servo.Direction.FORWARD);
        pixelRotLeft.setDirection(Servo.Direction.REVERSE);
        pixelRotRight.setDirection(Servo.Direction.FORWARD);
        slideRight.setDirection(DcMotor.Direction.FORWARD);
        slideLeft.setDirection((DcMotor.Direction.REVERSE));


        // Telemetry
        telemetry.addData(">", "Press Start to test." );
        telemetry.addData("Left Slide: ", slideLeft);
        telemetry.addData("Right Slide: ", slideRight);
        telemetry.addData("Arm Servo Left: ", armRotationLeftPosition);
        telemetry.addData("Arm Servo Right: ", armRotationRightPosition);
        telemetry.update();
        waitForStart();


        while(opModeIsActive()) {

            // Check gamepad for activation

            // rotate intake servos
            if (gamepad1.left_bumper) { // intake in
                intakeLeft.setPower(1);
                intakeRight.setPower(1);

            } else if (gamepad1.right_bumper) { //intake out
                intakeLeft.setPower(-1);
                intakeRight.setPower(-1);
            } else {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
            }

            // incrementally rotate pixel grabber servo to identify start and stop positions
            if (gamepad1.right_stick_y != 0) {
                pixelGrabberLeftPosition += gamepad1.right_stick_y * SCALE;
                pixelGrabberRightPosition += gamepad1.right_stick_y * SCALE;
                if (pixelGrabberLeftPosition >= MAX_POS) {
                    pixelGrabberLeftPosition = MAX_POS;
                }
                if (pixelGrabberLeftPosition <= MIN_POS) {
                    pixelGrabberLeftPosition = MIN_POS;
                }
                if (pixelGrabberRightPosition >= MAX_POS) {
                    pixelGrabberRightPosition = MAX_POS;
                }
                if (pixelGrabberRightPosition <= MIN_POS) {
                    pixelGrabberRightPosition = MIN_POS;
                }
            }
            // rotate pixel grabber servo to pickup and release
            if (gamepad1.a) { // press A to grab pixel
                pixelGrabberLeftPosition = 0.72;
                pixelGrabberRightPosition = 0.72;
            }
            if (gamepad1.b) { // press B to release pixel
                pixelGrabberLeftPosition = 0;
                pixelGrabberRightPosition = 0;
            }

            // incrementally rotate arm rotation servo
            if (gamepad1.left_stick_y != 0) {
                armRotationLeftPosition += -gamepad1.left_stick_y * SCALE;
                armRotationRightPosition += -gamepad1.left_stick_y * SCALE;
                if (armRotationLeftPosition >= MAX_POS) {
                    armRotationLeftPosition = MAX_POS;
                }
                if (armRotationLeftPosition <= MIN_POS_ARM) {
                    armRotationLeftPosition = MIN_POS_ARM;
                }
                if (armRotationRightPosition >= MAX_POS) {
                    armRotationRightPosition = MAX_POS;
                }
                if (armRotationRightPosition <= MIN_POS_ARM) {
                    armRotationRightPosition = MIN_POS_ARM;
                }
            }
            // incrementally rotate pixel rotation servo
            if (gamepad1.left_stick_x != 0) {
                pixelRotateLeftPosition += -gamepad1.left_stick_x * SCALE;
                pixelRotateRightPosition += -gamepad1.left_stick_x * SCALE;
                if (pixelRotateLeftPosition >= MAX_POS_PIXEL) {
                    pixelRotateLeftPosition = MAX_POS_PIXEL;
                }
                if (pixelRotateLeftPosition <= MIN_POS_PIXEL) {
                    pixelRotateLeftPosition = MIN_POS_PIXEL;
                }
                if (pixelRotateRightPosition >= MAX_POS_PIXEL) {
                    pixelRotateRightPosition = MAX_POS_PIXEL;
                }
                if (pixelRotateRightPosition <= MIN_POS_PIXEL) {
                    pixelRotateRightPosition = MIN_POS_PIXEL;
                }
            }
            // manual slide control
            if (gamepad1.dpad_up){
                slideLeft.setPower(.25);
                slideRight.setPower(.25);
            }
            //limit switch left
            if (limitSwitchLeft.isPressed() && gamepad1.dpad_down){
                slideLeft.setPower(0);
            }
            // limit switch right
            if (limitSwitchRight.isPressed() && gamepad1.dpad_down){
                slideRight.setPower(0);
            }


            // Set the servos to the new positions
            grabberServoLeft.setPosition(pixelGrabberLeftPosition);
            grabberServoRight.setPosition(pixelGrabberRightPosition);
            armRotLeft.setPosition(armRotationLeftPosition);
            armRotRight.setPosition(armRotationRightPosition);
            pixelRotLeft.setPosition(pixelRotateLeftPosition);
            pixelRotRight.setPosition(pixelRotateRightPosition);

            // Display the current value
            telemetry.addData("Pixel Grabber Left Servo Position", "%5.2f", grabberServoLeft.getPosition());
            telemetry.addData("Pixel Grabber Right Servo Position", "%5.2f", grabberServoRight.getPosition());
            telemetry.addData("Arm Rotate Left Servo Position", "%5.2f", armRotLeft.getPosition());
            telemetry.addData("Arm Rotate Right Servo Position", "%5.2f", armRotRight.getPosition());
            telemetry.addData("Pixel Rotate Left Servo Position", "%5.2f", pixelRotLeft.getPosition());
            telemetry.addData("Pixel Rotate Right Servo Position", "%5.2f", pixelRotRight.getPosition());


            telemetry.addData(">", "Press Stop to end test.");

            telemetry.update();
        }

    }
 }

