package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.concurrent.TimeUnit;
@Config
public class  UniversalControlClass {
    LinearOpMode opMode;
    //TODO: Create Motors, sensors, and servos
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor rightRear;
    BNO055IMU imu;

    //TODO: set universal variables (public static to make available in dashboard
    boolean IsDriverControl;
    boolean IsFieldCentric;

    //TODO: Add any other specification variables
    public UniversalControlClass(boolean isDriverControl, boolean isFieldCentric, LinearOpMode opMode) {
        this.IsDriverControl = isDriverControl;
        this.IsFieldCentric = isFieldCentric;
        this.opMode = opMode;
    }

    public void Init (HardwareMap hardwareMap) {
        //TODO: hardware map all servos, motors, sensors, and cameras
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        //TODO: set motor direction, zero power brake behavior, stop and reset encoders, etc
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void ServoIntake() {
        // TODO: AIDAN continuous rotation servo intake
    }

    public void IntakeDistanceStop() {
        //TODO: JACOB stop intake servos when distance
    }

    public void SlidesUp(){
        //TODO: CLAIRE slides up w/ limit switch
    }

    public void SlidesDown() {
        //TODO: CLAIRE slides down w/ limit switch
    }

    public void LightControl() {
        //TODO: AIDAN Blinkin module with color detection
    }

    public void DetectTeamArt() {
        //TODO: JACOB detect team art location and set variable for location
    }

    public void LaunchAirplane() {
        //TODO: UNASSIGNED Launch plane
    }

    public void Hang() {
        //TODO: UNASSIGNED Hang from bar
    }

    public void TransferDeliver() {
        //TODO: CLAIRE grab, rotate, drop mechanism for outtake
        //theoretical transfer for now, just worry about open and close servos for grab and drop
    }

    public void driveControlsRobotCentric() {
        double y = opMode.gamepad2.left_stick_y;
        double x = -opMode.gamepad2.left_stick_x * 1.1;
        double rx = opMode.gamepad2.right_stick_x;

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

    public void driveControlsRobotCentricKID() {
        double y = opMode.gamepad2.left_stick_y;
        double x = -opMode.gamepad2.left_stick_x * 1.1;
        double rx = opMode.gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower*.25);
        leftRear.setPower(backLeftPower*.25);
        rightFront.setPower(frontRightPower*.25);
        rightRear.setPower(backRightPower*.25);
    }

    public void driveControlsFieldCentric() {
        double y = -opMode.gamepad2.left_stick_y;
        double x = opMode.gamepad2.left_stick_x * 1.1;
        double rx = opMode.gamepad2.right_stick_x;

        double botHeading = -imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }

//    public void WaitForTrajectoryToFinish(SampleMecanumDrive drive) {
//        while(opMode.opModeIsActive() && !opMode.isStopRequested() && drive.isBusy()) {
//            if(drive != null){
//                drive.update();
//                opMode.telemetry.addData("stack pos", STACK_POS);
//                opMode.telemetry.addData("slide current pos", slideOne.getCurrentPosition());
//                opMode.telemetry.update();
//            }
//            if (IsDriverControl) {
//                if(IsFieldCentric) driveControlsFieldCentric();
//                if(!IsFieldCentric) driveControlsRobotCentric();
//            }
//        }
//    }
//
//    public void WaitForSlides(SampleMecanumDrive drive) {
//        while ((slideOne.isBusy()) && (slideTwo.isBusy()) && (!opMode.isStopRequested())) {
//            if(drive != null) {
//                drive.update();
////                Pose2d poseEstimate = drive.getPoseEstimate();
////                opMode.telemetry.addData("y", poseEstimate.getX());
////                opMode.telemetry.addData("x", poseEstimate.getY());
////                opMode.telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
////                opMode.telemetry.addData("front distance", rangeClaw);
//                opMode.telemetry.addData("stack pos", STACK_POS);
//                opMode.telemetry.addData("slide current pos", slideOne.getCurrentPosition());
//                opMode.telemetry.update();
//            }
//            if (IsDriverControl) {
//                if(IsFieldCentric) driveControlsFieldCentric();
//                if(!IsFieldCentric) driveControlsRobotCentric();
//            }
//        }
//    }
//
//    public void SpecialSleep(SampleMecanumDrive drive, long milliseconds) {
//        for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(milliseconds); stop>System.nanoTime();) {
//            if (!opMode.opModeIsActive() || opMode.isStopRequested() ) return;
//            if(drive != null) {
//                drive.update();
////                Pose2d poseEstimate = drive.getPoseEstimate();
////                opMode.telemetry.addData("y", poseEstimate.getX());
////                opMode.telemetry.addData("x", poseEstimate.getY());
////                opMode.telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
////                opMode.telemetry.addData("front distance", rangeClaw);
//                opMode.telemetry.addData("stack pos", STACK_POS);
//                opMode.telemetry.addData("slide current pos", slideOne.getCurrentPosition());
//                opMode.telemetry.update();
//            }
//            if (IsDriverControl) {
//                if(IsFieldCentric) driveControlsFieldCentric();
//                if(!IsFieldCentric) driveControlsRobotCentric();
//            }
//        }
//    }
//    public void SpecialSleepTraj(SampleMecanumDrive drive, long milliseconds) {
//        for (long stop = System.nanoTime()+ TimeUnit.MILLISECONDS.toNanos(milliseconds); stop>System.nanoTime();) {
//            if (!opMode.opModeIsActive() || opMode.isStopRequested() )
//                if(!drive.isBusy()) return;
//            if(drive != null) {
//                drive.update();
////                Pose2d poseEstimate = drive.getPoseEstimate();
////                opMode.telemetry.addData("y", poseEstimate.getX());
////                opMode.telemetry.addData("x", poseEstimate.getY());
////                opMode.telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
////                opMode.telemetry.addData("front distance", rangeClaw);
//                opMode.telemetry.addData("stack pos", STACK_POS);
//                opMode.telemetry.addData("slide current pos", slideOne.getCurrentPosition());
//                opMode.telemetry.update();
//            }
//            if (IsDriverControl) {
//                if(IsFieldCentric) driveControlsFieldCentric();
//                if(!IsFieldCentric) driveControlsRobotCentric();
//            }
//        }
//    }
}

