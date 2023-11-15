package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
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
public class TestOpMode extends LinearOpMode {
    UniversalControlClass control = new UniversalControlClass(true, false, this);
    public void runOpMode(){
        control.Init(hardwareMap);
        control.HuskyLensInit();

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            control.driveControlsRobotCentric();

            if (gamepad1.dpad_up){
                control.SlidesUp();
            }
            else if (gamepad1.dpad_down){
                control.SlidesDown();
            }
            else{
                control.rightSlide.setPower(0);
                control.leftSlide.setPower(0);
            }

            if (gamepad1.right_bumper){
                control.ServoIntake();
            }
            else if (gamepad1.left_bumper){
                control.ServoOuttake();
            }
            else{
                control.intakeRight.setPower(0);
                control.intakeLeft.setPower(0);
            }
        }
    }
}
