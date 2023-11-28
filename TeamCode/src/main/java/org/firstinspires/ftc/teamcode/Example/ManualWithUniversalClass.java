package org.firstinspires.ftc.teamcode.Example;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.UniversalControlClass;
@TeleOp(name = "Manual Control", group = "Test")
public class ManualWithUniversalClass extends LinearOpMode {
    UniversalControlClass theRobot = new UniversalControlClass(true, true, this);
    static final double SCALE = 0.001;
    public void runOpMode() {
        theRobot.Init(this.hardwareMap);
        theRobot.ShowSlideTelemetry();
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            theRobot.driveControlsFieldCentric();
            if (gamepad2.left_bumper) { // intake in
                theRobot.ServoIntake();

            } else if (gamepad2.right_bumper) { //intake out
                theRobot.ServoOuttake();
            } else {
                theRobot.ServoStop();
            }

            if (gamepad2.dpad_up) {  // slides up and down with limit switches
                theRobot.SetSlidePower(.25);
            }else if (gamepad2.dpad_down){
                theRobot.SetSlidePower(-.25);
            }else {
                theRobot.SetSlidePower(0);
            }

            if (gamepad2.left_stick_y != 0){ // wrist movement
                theRobot.ArmWrist(theRobot.getWristPosition() + -gamepad2.left_stick_y * SCALE);
            }

            if (gamepad2.right_stick_y != 0){ // whole arm movement
                theRobot.WholeArmRot(theRobot.getWholeArmPosition() + -gamepad2.right_stick_y * SCALE);
            }

            if (gamepad2.a){ // grab left pixel
                theRobot.GrabLeft();
            }else if (gamepad2.b){ // release left pixel
                theRobot.ReleaseLeft();
            }else if (gamepad2.x) { // grab right pixel
                theRobot.GrabRight();
            }else if (gamepad2.y){ // release right pixel
                theRobot.ReleaseRight();
            }


            theRobot.ShowSlideTelemetry();
            telemetry.update();
        }
    }
}
