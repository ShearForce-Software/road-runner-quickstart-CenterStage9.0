package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

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
    TouchSensor leftSlideLimit;
    TouchSensor rightSlideLimit;
    CRServo intakeLeft;
    CRServo intakeRight;
    DistanceSensor leftHopper;
    DistanceSensor rightHopper;
    RevColorSensorV3 leftColorSensor;
    RevColorSensorV3 rightColorSensor;
    HuskyLens huskyLens;
    BNO055IMU imu;
    Servo   grabberLeft;
    Servo   grabberRight;
    Servo armRotRight;
    Servo armRotLeft;
    Servo pixelRotRight;
    Servo pixelRotLeft;
    Servo droneLauncher;
    RevBlinkinLedDriver.BlinkinPattern Blinken_left_pattern;
    RevBlinkinLedDriver.BlinkinPattern Blinken_right_pattern;
    RevBlinkinLedDriver blinkinLedDriverLeft;
    RevBlinkinLedDriver blinkinLedDriverRight;


    //TODO: set universal variables (public static to make available in dashboard
    boolean IsDriverControl;
    boolean IsFieldCentric;
    boolean isAllianceBlue;
    int hopperDistance = 5;
    double  grabberPosition = 0; // Start at minimum rotational position
    int spikeBound = 160;
    int autoPosition;
    public static double grabPosition = 0.5;
    public static double dropPosition = 0;

    public static final double SLIDE_POWER   = 0.50;
    public static final int SLIDE_MAX_HEIGHT = 500;
    public static final int SLIDE_MED_HEIGHT = 250;
    public static final int SLIDE_MIN_HEIGHT = 0;
    public double wristPosition = 0.0;
    public double wholeArmPosition = 0.04;
    static final double MAX_WRIST_POS = 1.0;
    static final double MIN_WRIST_POS = 0.0;
    static final double MAX_WHOLE_ARM_POS = 1.0;
    static final double MIN_WHOLE_ARM_POS = 0.04;
    //NAV TO TAG VARIABLES
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    double rangeError = 0;
    double yawError = 0;
    private double slidePower = 0.0;

    //TODO: Add any other specification variables
    public UniversalControlClass(boolean isDriverControl, boolean isFieldCentric, LinearOpMode opMode) {
        this.IsDriverControl = isDriverControl;
        this.IsFieldCentric = isFieldCentric;
        this.opMode = opMode;
    }

    public void setDriverControlFieldCentric(){
        IsFieldCentric = true;
    }

    public void setDriverControlRobotCentric(){
        IsFieldCentric = false;
    }

    public void setAllianceColor(boolean isBlue){
        isAllianceBlue = isBlue;
    }


    public void WebcamInit (HardwareMap hardwareMap){
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    public void NavToTag(){
        desiredTag  = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == DESIRED_TAG_ID) ){
                desiredTag = detection;
                rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                yawError        = desiredTag.ftcPose.yaw;
                break;  // don't look any further.
            } else {
                opMode.telemetry.addData("Unknown Target - ", "No Tag Detected");
                // set some range and yaw error
            }
        }
    }
    public void Init (HardwareMap hardwareMap) {
        //TODO: hardware map all servos, motors, sensors, and cameras
        leftFront = hardwareMap.get(DcMotor.class, "leftFront_leftOdometry");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront_rightOdometry");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        grabberLeft = hardwareMap.get(Servo.class, "pixelGrabberLeft");
        grabberRight = hardwareMap.get(Servo.class, "pixelGrabberRight");
        armRotLeft = hardwareMap.get(Servo.class, "armRotateLeft");
        armRotRight = hardwareMap.get(Servo.class, "armRotateRight");
        pixelRotLeft = hardwareMap.get(Servo.class, "pixelRotateLeft");
        pixelRotRight = hardwareMap.get(Servo.class, "pixelRotateRight");
        droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        leftSlideLimit = hardwareMap.get(TouchSensor.class, "leftSlideLimit");
        rightSlideLimit = hardwareMap.get(TouchSensor.class, "rightSlideLimit");
        //InitBlinkin(hardwareMap);
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens1");

        //TODO: set motor direction, zero power brake behavior, stop and reset encoders, etc
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        armRotLeft.setDirection(Servo.Direction.REVERSE);
        armRotRight.setDirection(Servo.Direction.FORWARD);
        pixelRotLeft.setDirection(Servo.Direction.REVERSE);
        grabberLeft.setDirection(Servo.Direction.REVERSE);
        grabberRight.setDirection(Servo.Direction.FORWARD);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setDirection(DcMotor.Direction.FORWARD);
        leftSlide.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void AutoStartPos(){
        grabberLeft.setPosition(0);
        grabberRight.setPosition(0);
        armRotLeft.setPosition(.16);
        armRotRight.setPosition(.16);
        pixelRotLeft.setPosition(.06);
        pixelRotRight.setPosition(.06);
    }

    public void GrabPixels(){
        grabberRight.setPosition(.72);
        grabberLeft.setPosition(.72);
    }
    public void DropOnLine(){
        armRotLeft.setPosition(.96);
        armRotRight.setPosition(.96);
        opMode.sleep(500);
        grabberLeft.setPosition(0);
        opMode.sleep(500);
    }

    public void SafeStow(){
        grabberLeft.setPosition(.72);
        grabberRight.setPosition(.72);
        armRotLeft.setPosition(.16);
        armRotRight.setPosition(.16);
        pixelRotLeft.setPosition(.06);
        pixelRotRight.setPosition(.06);
    }
    public void ManualStartPos(){
        armRotLeft.setPosition(.076);
        armRotRight.setPosition(.076);
        pixelRotLeft.setPosition(.66);
        pixelRotRight.setPosition(.66);
    }

    public void GrabPixelPos(){ // in center of pixels
        armRotLeft.setPosition(.04);
        armRotRight.setPosition(.04);
        pixelRotLeft.setPosition(.63);
        pixelRotRight.setPosition(.63);

    }

    public void ReadyToLiftSlides(){ // slight move before lifting slides
        armRotLeft.setPosition(.76);
        armRotRight.setPosition(.76);
        pixelRotLeft.setPosition(.63);
        pixelRotRight.setPosition(.63);

    }

    public void DeliverPixelToBoardPos(){
        armRotLeft.setPosition(.86);
        armRotRight.setPosition(.86);
        pixelRotLeft.setPosition(.19);
        pixelRotRight.setPosition(.19);

    }

    public void InitBlinkin(HardwareMap hardwareMap) {
        blinkinLedDriverLeft = hardwareMap.get(RevBlinkinLedDriver.class,"leftBlinkin");
        blinkinLedDriverRight = hardwareMap.get(RevBlinkinLedDriver.class,"rightBlinkin");
        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensorLeft");

        blinkinLedDriverLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    }
    public void ArmWrist(double position){
        if (position > MAX_WRIST_POS){
            wristPosition = MAX_WRIST_POS;
        }else if(position < MIN_WRIST_POS){
            wristPosition = MIN_WRIST_POS;
        }else{
            wristPosition = position;
        }
        pixelRotLeft.setPosition(wristPosition);
        pixelRotRight.setPosition(wristPosition);
    }
    public double getWristPosition(){
        return wristPosition;
    }
    public void WholeArmRot(double position){
    if (position > MAX_WHOLE_ARM_POS){
            wholeArmPosition = MAX_WHOLE_ARM_POS;
        }else if(position < MIN_WHOLE_ARM_POS){
            wholeArmPosition = MIN_WHOLE_ARM_POS;
        }else{
            wholeArmPosition = position;
        }
        armRotLeft.setPosition(wholeArmPosition);
        armRotRight.setPosition(wholeArmPosition);
    }
    public double getWholeArmPosition(){
        return wholeArmPosition;
    }
    public void GrabLeft(){
        grabberLeft.setPosition(.72);
    }
    public void GrabRight(){
        grabberRight.setPosition(.72);
    }
    public void ReleaseLeft(){
        grabberLeft.setPosition(0);
    }
    public void ReleaseRight(){
        grabberRight.setPosition(0);
    }

    public void ServoIntake() {
        intakeRight.setPower(1.0);
        intakeLeft.setPower(1.0);
    }
    public void ServoOuttake() {
        intakeRight.setPower(-1);
        intakeLeft.setPower(-1);
    }
    public void ServoStop(){
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
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

    public void SlidesMedium(){
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setTargetPosition(SLIDE_MED_HEIGHT);
        rightSlide.setTargetPosition(SLIDE_MED_HEIGHT);
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
    public void ManualSlide(double power){
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }
    public void SetSlidePower(double power){
        //TODO: CLAIRE slides w/ limit switch
        if ((leftSlideLimit.isPressed() && power < 0) || (rightSlideLimit.isPressed() && power < 0))
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
    public void ShowSlideTelemetry(){
        opMode.telemetry.addData("Left Slide: ", leftSlide.getCurrentPosition());
        opMode.telemetry.addData("Right Slide: ", rightSlide.getCurrentPosition());
        opMode.telemetry.addData("Arm Servo Left: ", armRotLeft.getPosition());
        opMode.telemetry.addData("Arm Servo Right: ", armRotRight.getPosition());
        opMode.telemetry.addData("Wrist Position: ", wristPosition);
        opMode.telemetry.addData("Whole Arm Position: ", wholeArmPosition);
    }

    }
    public void PickupRoutine(){

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
//    }

    public void ColorDetect(){

        //double rightColor = rightColorSensor.getLightDetected();

    }
    public void showColorSensorTelemetry(){
        int leftColor = leftColorSensor.getNormalizedColors().toColor();
        opMode.telemetry.addData("leftColorNorm: ", leftColor);
        opMode.telemetry.addData("leftColor(red): ", leftColorSensor.red());
        opMode.telemetry.addData("leftColor(green): ", leftColorSensor.green());
        opMode.telemetry.addData("leftColor(blue): ", leftColorSensor.blue());
        //opMode.telemetry.addData("rightColor: ", rightColor);
        opMode.telemetry.addData("leftColorNorm(red): ", leftColorSensor.getNormalizedColors().red);
        opMode.telemetry.addData("leftColorNorm(green): ", leftColorSensor.getNormalizedColors().green);
        opMode.telemetry.addData("leftColorNorm(blue): ", leftColorSensor.getNormalizedColors().blue);
        int red = leftColorSensor.red();
        int green = leftColorSensor.green();
        int blue = leftColorSensor.blue();
        // Check for White Pixel
        if(red < 4000 && red > 1000 && green < 6000 && green > 3000 && blue < 7000 && blue > 3000) {
            opMode.telemetry.addData("Left: ", "is white");
        }
        // Check for yellow pixel
        else if(red < 2500 && red > 1000 && green < 3500 && green > 1500 && blue < 1000 && blue >0 )
        {
            opMode.telemetry.addData("Left: ", "is yellow");
        }
        // Check for green pixel
        else if(red < 1000 && red > 0 && green < 6000 && green > 1500 && blue < 1000 && blue >0 )
        {
            opMode.telemetry.addData("Left: ", "is green");
        }
        // Check for purple pixel
        else if(red < 3500 && red > 1000 && green < 4000 && green > 2000 && blue < 7000 && blue > 3500 )
        {
            opMode.telemetry.addData("Left: ", "is purple");
        }
        else {
            opMode.telemetry.addData("Left: ", "unknown");
        }
    }
    public void SetBlinkinToPixelColor()
    {
        int red = leftColorSensor.red();
        int green = leftColorSensor.green();
        int blue = leftColorSensor.blue();
        // Check for White Pixel
        if(red < 4000 && red > 1000 && green < 6000 && green > 3000 && blue < 7000 && blue > 3000) {
           Set_Blinkin_Left_White();
        }
        // Check for yellow pixel
        else if(red < 2500 && red > 1000 && green < 3500 && green > 1500 && blue < 1000 && blue >0 )
        {
            Set_Blinkin_Left_Yellow();
        }
        // Check for green pixel
        else if(red < 1000 && red > 0 && green < 6000 && green > 1500 && blue < 1000 && blue >0 )
        {
            Set_Blinkin_Left_Green();
        }
        // Check for purple pixel
        else if(red < 3500 && red > 1000 && green < 4000 && green > 2000 && blue < 7000 && blue > 3500 )
        {
            Set_Blinkin_Left_Violet();
        }
        else {
            Set_Blinkin_Left_Black();
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
           // opMode.telemetry.addData("Blinkin Right: ", Blinken_right_pattern.toString());
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
    public void DetectTeamArtBlue() {
        //TODO: MADDIE/JACOB detect team art location and set variable for location
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (blocks.length > 0){
            int xVal = blocks[0].x;
            opMode.telemetry.addData("Team Art Detected: ", true);
            opMode.telemetry.addData("Team Art X position: ", xVal);
            if (xVal < spikeBound){
                autoPosition = 2;
                DESIRED_TAG_ID = 2;
            }
            else if (xVal > spikeBound){
                autoPosition = 3;
                DESIRED_TAG_ID = 3;
            }
            else
            {
                autoPosition = 1;
                DESIRED_TAG_ID = 1;
            }
            opMode.telemetry.addData("Auto position: ", autoPosition);
        }
        else{
            //pick a spot
            opMode.telemetry.addData("!!Team Art NOT DETECTED!! ", "DEFAULT TO CENTER");
            autoPosition = 2;
            DESIRED_TAG_ID = 2;
        }
    }

    public void DetectTeamArtRed() {
        //TODO: MADDIE/JACOB detect team art location and set variable for location
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (blocks.length > 0){
            int xVal = blocks[0].x;
            opMode.telemetry.addData("Team Art Detected: ", true);
            opMode.telemetry.addData("Team Art X position: ", xVal);
            if (xVal < spikeBound){
                autoPosition = 2;
                DESIRED_TAG_ID = 5;
            }
            else if (xVal > spikeBound){
                autoPosition = 3;
                DESIRED_TAG_ID = 6;
            }
            else
            {
                autoPosition = 1;
                DESIRED_TAG_ID = 4;
            }
            opMode.telemetry.addData("Auto position: ", autoPosition);
        }
        else{
            //pick a spot
            opMode.telemetry.addData("!!Team Art NOT DETECTED!! ", "DEFAULT TO CENTER");
            autoPosition = 2;
            DESIRED_TAG_ID = 5;
        }
    }
    public void LaunchAirplane() {
        //TODO: UNASSIGNED Launch plane
        droneLauncher.setPosition(1);
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
    public void runDriveControls(){
        if (this.IsFieldCentric) {
            driveControlsFieldCentric();
        }
        else {
            driveControlsRobotCentric();
        }
    }
    public void driveControlsRobotCentric() {
        double y = opMode.gamepad1.left_stick_y;
        double x = -opMode.gamepad1.left_stick_x * 1.1;
        double rx = opMode.gamepad1.right_stick_x;

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
        double y = -opMode.gamepad1.left_stick_y;
        double x = opMode.gamepad1.left_stick_x * 1.1;
        double rx = opMode.gamepad1.right_stick_x;

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

