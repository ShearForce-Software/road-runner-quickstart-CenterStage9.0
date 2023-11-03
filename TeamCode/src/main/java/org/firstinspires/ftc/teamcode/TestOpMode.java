package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="TestOpMode", group="Linear OpMode")
public class TestOpMode extends UniversalControlClass {
    double y = opMode.gamepad2.left_stick_y;
    double x = -opMode.gamepad2.left_stick_x * 1.1;
    double rx = opMode.gamepad2.right_stick_x;
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor rightRear;
    DcMotor rightSlide;
    DcMotor leftSlide;

    public TestOpMode(boolean isDriverControl, boolean isFieldCentric, LinearOpMode opMode) {
        super(isDriverControl, isFieldCentric, opMode);
    }

    public void runOpMode(){
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        leftRear  = hardwareMap.get(DcMotor.class, "left_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        telemetry.update();
        opMode.waitForStart();

        while (opMode.opModeIsActive()) {
            double y = opMode.gamepad2.left_stick_y;
            double x = -opMode.gamepad2.left_stick_x * 1.1;
            double rx = opMode.gamepad2.right_stick_x;
        }
    }
}
