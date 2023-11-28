package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name="TestOpMode", group="Linear OpMode")
public class TestOpMode extends LinearOpMode {
    //UniversalControlClass control = new UniversalControlClass(true, false, this);

    Servo grabberLeft;
    Servo grabberRight;
    DcMotor leftSlide;
    DcMotor rightSlide;
    CRServo leftIntake;
    CRServo rightIntake;
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor rightRear;

    public void runOpMode(){
        //control.Init(hardwareMap);
        //control.HuskyLensInit();
        leftFront = hardwareMap.get(DcMotor.class, "leftFront_leftOdometry");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront_rightOdometry");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        grabberLeft = hardwareMap.get(Servo.class, "leftGrabber");
        grabberRight = hardwareMap.get(Servo.class, "rightGrabber");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grabberLeft.setPosition(0);
        grabberRight.setPosition(0);

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            //control.driveControlsRobotCentric();

            //SLIDE CONTROL
            if (gamepad1.dpad_up){
                leftSlide.setPower(1);
                rightSlide.setPower(1);
            }
            else if (gamepad1.dpad_down){
                leftSlide.setPower(-1);
                rightSlide.setPower(-1);
                //control.SlidesDown();
            }
            else{
                leftSlide.setPower(0);
                rightSlide.setPower(0);
                //control.rightSlide.setPower(0);
                //control.leftSlide.setPower(0);
            }

            //GRAB/DROP
            if (gamepad1.right_bumper){
                grabberLeft.setPosition(.75);
                grabberRight.setPosition(.75);
            }
            else if (gamepad1.left_bumper){
                grabberLeft.setPosition(0);
                grabberRight.setPosition(0);
            }

            //INTAKE/OUTTAKE
            if (gamepad1.right_trigger!= 0){
                //control.ServoIntake();
                leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
                leftIntake.setPower(1.0);
                rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
                rightIntake.setPower(1.0);
            }
            else if (gamepad1.left_trigger!= 0){
                //control.ServoOuttake();
                leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
                leftIntake.setPower(1.0);
                rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
                rightIntake.setPower(1.0);
            }
            else{
                //control.intakeRight.setPower(0);
                //control.intakeLeft.setPower(0);
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }

            telemetry.addData("Dpad Up = Slides Up", ", Dpad Down = Slides Down");
            telemetry.addData("Right Trigger = Spin In", ", Left Trigger = Spin Out");
            telemetry.addData("Right Bumper = Grab", ", Left Bumper = Drop");
        }
    }
    public void driveControlsRobotCentric() {
        double y = gamepad2.left_stick_y;
        double x = -gamepad2.left_stick_x * 1.1;
        double rx = gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }
}
