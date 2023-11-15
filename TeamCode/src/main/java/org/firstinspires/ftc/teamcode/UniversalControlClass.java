package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;

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
    DcMotor rightSlide;
    DcMotor leftSlide;
    DigitalChannel SlideLimit;
    CRServo intakeLeft;
    CRServo intakeRight;
    DistanceSensor leftHopper;
    DistanceSensor rightHopper;
    RevColorSensorV3 leftColorSensor;
    RevColorSensorV3 rightColorSensor;
    HuskyLens huskyLens;
    BNO055IMU imu;
    Servo   grabberServo1;
    RevBlinkinLedDriver.BlinkinPattern Blinken_left_pattern;
    RevBlinkinLedDriver.BlinkinPattern Blinken_right_pattern;
    RevBlinkinLedDriver blinkinLedDriverLeft;
    RevBlinkinLedDriver blinkinLedDriverRight;


    //TODO: set universal variables (public static to make available in dashboard
    boolean IsDriverControl;
    boolean IsFieldCentric;
    int hopperDistance = 5;
    double  grabberPosition = 0; // Start at minimum rotational position
    int leftSpikeBound = 100;
    int rightSpikeBound = 200;
    int autoPosition;
    public static double grabPosition = 0.5;
    public static double dropPosition = 0;

    public static final double SLIDE_POWER   = 0.50;
    public static final int SLIDE_MAX_HEIGHT = 500;
    public static final int SLIDE_MIN_HEIGHT = 0;

    private double slidePower = 0.0;

    //TODO: Add any other specification variables
    public UniversalControlClass(boolean isDriverControl, boolean isFieldCentric, LinearOpMode opMode) {
        this.IsDriverControl = isDriverControl;
        this.IsFieldCentric = isFieldCentric;
        this.opMode = opMode;
    }

    public void Init (HardwareMap hardwareMap) {
       /* //TODO: hardware map all servos, motors, sensors, and cameras
        leftFront = hardwareMap.get(DcMotor.class, "leftFront_leftOdometry");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront_rightOdometry");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        grabberServo1 = hardwareMap.get(Servo.class, "grabber");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        SlideLimit = hardwareMap.get(DigitalChannel.class, "SlideLimit");
*/
        blinkinLedDriverLeft = hardwareMap.get(RevBlinkinLedDriver.class,"leftBlinkin");
        blinkinLedDriverRight = hardwareMap.get(RevBlinkinLedDriver.class,"rightBlinkin");

        blinkinLedDriverLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensorLeft");
     /*   //TODO: set motor direction, zero power brake behavior, stop and reset encoders, etc
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SlideLimit.setMode(DigitalChannel.Mode.INPUT);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

      */
    }

    public void ServoIntake() {
        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setPower(1.0);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeft.setPower(1.0);
    }
    public void ServoOuttake() {
        intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeRight.setPower(-1);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeft.setPower(-1);
    }

    public void IntakeDistanceStop() {
        //TODO: JACOB Find hopper distance
        if((leftHopper.getDistance(DistanceUnit.MM) < hopperDistance) && (rightHopper.getDistance(DistanceUnit.MM) < hopperDistance)){
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }
    }

    public void SlidesUp(){
        //TODO: CLAIRE find SLIDE_MAX_HEIGHT
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setTargetPosition(SLIDE_MAX_HEIGHT);
        rightSlide.setTargetPosition(SLIDE_MAX_HEIGHT);
        SetSlidePower(SLIDE_POWER);
    }
    public void SlidesDown() {
        //TODO: CLAIRE find SLIDE_MIN_HEIGHT
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setTargetPosition(SLIDE_MIN_HEIGHT);
        rightSlide.setTargetPosition(SLIDE_MIN_HEIGHT);
        SetSlidePower(-1*SLIDE_POWER);
    }
    private void SetSlidePower(double power){
        //TODO: CLAIRE slides w/ limit switch
        if (SlideLimit.getState() == true && power > 0)
        {
            slidePower = 0;
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        else
        {
            slidePower = power;
        }
        leftSlide.setPower(slidePower);
        rightSlide.setPower(slidePower);
    }

//    CLAIRE- it seems like this method is unused? you check the slide limit in the SetSlidePower method.
//    public void CheckForSlideLimit()
//    {
//        if (SlideLimit.getState() == false && slidePower < 0)
//        {
//            slidePower = 0;
//            leftSlide.setPower(0);
//            rightSlide.setPower(0);
//            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
//    }q

    public void ColorDetect(){
        double leftColor = leftColorSensor.getLightDetected();
       // double rightColor = rightColorSensor.getLightDetected();
        opMode.telemetry.addData("leftColor: ", leftColor);
       // opMode.telemetry.addData("rightColor: ", rightColor);
    }
    public void LightControl() {
    }
        //TODO: AIDAN Blinkin module with color detection
        public void SetLeftToColor(int number, int side)
        {
            RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            switch (number)
            {
                case 1:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                    break;
                case 2:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                    break;
                case 3:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                    break;
                case 4:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                    break;
                default:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                    break;
            }
            if (side == 1)
            {
                Blinken_left_pattern= pattern;
                blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
            }
            else
            {
                Blinken_right_pattern = pattern;
                blinkinLedDriverRight.setPattern(Blinken_right_pattern);
            }
        }
        public void Set_Blinkin_Left_Green()
        {
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }
    public void Set_Blinkin_Right_Green()
    {
        Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        blinkinLedDriverRight.setPattern(Blinken_right_pattern);
    }
    public void Set_Blinkin_Left_Violet()
    {
        Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
        blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
    }
    public void Set_Blinkin_Right_Violet()
    {
        Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
        blinkinLedDriverRight.setPattern(Blinken_right_pattern);
    }
    public void Set_Blinkin_Left_Yellow()
    {
        Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
    }
    public void Set_Blinkin_Right_Yellow()
    {
        Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        blinkinLedDriverRight.setPattern(Blinken_right_pattern);
    }
    public void Set_Blinkin_Left_White()
    {
        Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
    }
    public void Set_Blinkin_Right_White()
    {
        Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        blinkinLedDriverRight.setPattern(Blinken_right_pattern);
    }
    public void Set_Blinkin_Left_Black()
    {
        Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
    }
    public void Set_Blinkin_Right_Black()
    {
        Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        blinkinLedDriverRight.setPattern(Blinken_right_pattern);
    }
    public void Set_Blinkin_Left_Red()
    {
        Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
    }
    public void Set_Blinkin_Right_Red()
    {
        Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        blinkinLedDriverRight.setPattern(Blinken_right_pattern);
    }
    public void Set_Blinkin_Left_Blue()
    {
        Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
    }
    public void Set_Blinkin_Right_Blue()
    {
        Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
    }
    public void Set_Blinkin_Left_ShearForce()
    {
        Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
        blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
    }
    public void Set_Blinkin_Right_ShearForce()
    {
        Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
        blinkinLedDriverRight.setPattern(Blinken_right_pattern);
    }
        public void Show_Blinkin_Telemetry()
        {
            opMode.telemetry.addData("Blinkin Left: ", Blinken_left_pattern.toString());
          //  opMode.telemetry.addData("Blinkin Right: ", Blinken_right_pattern.toString());
        }
    public void HuskyLensInit(){
        if (!huskyLens.knock()) {
            opMode.telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            opMode.telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
        //TODO: is there anything we have to do to import model?
        opMode.telemetry.update();
    }
    public void DetectTeamArt() {
        //TODO: MADDIE/JACOB detect team art location and set variable for location
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (blocks.length <0){
            int xVal = blocks[0].x;
            opMode.telemetry.addData("Team Art Detected: ", true);
            opMode.telemetry.addData("Team Art X position: ", xVal);
            //x value ranges from left to right 0 to 320, with 160 being the center
            //create variables for x min/max values for each spike mark location 1,2,3
            //determine position and assign variable for drive in autonomous
            if (xVal < leftSpikeBound){
                autoPosition = 1;
            }
            else if ((xVal >= leftSpikeBound) && (xVal <= rightSpikeBound)){
                autoPosition = 2;

            }
            else if (xVal > rightSpikeBound){
                autoPosition = 3;
            }
            opMode.telemetry.addData("Auto position: ", autoPosition);
        }
        else{
            //pick a spot
            opMode.telemetry.addData("!!Team Art NOT DETECTED!! ", "DEFAULT TO CENTER");
            autoPosition = 2;
        }
    }

    public void LaunchAirplane() {
        //TODO: UNASSIGNED Launch plane
    }

    public void Hang() {
        //TODO: UNASSIGNED Hang from bar
    }

//    DISABLED FOR NOW, SO ROBOT WON'T BE MAD
//    public void TransferDeliver() {
//        //TODO: CLAIRE grab, rotate, drop mechanism for outtake
//        //theoretical transfer for now, just worry about open and close servos for grab and drop
//        //close servo to grab
//        setGrabberPosition(grabPosition);
//        //TODO rotate something
//        //open servo to drop
//        setGrabberPosition(dropPosition);
//    }
//    public void setGrabberPosition (double position)
//    {
//        grabberPosition = position;
//        grabberServo1.setPosition(grabberPosition);
//        opMode.telemetry.addData("Grabber Position", "%5.2f", grabberPosition);
//        opMode.telemetry.update();
//    }

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

