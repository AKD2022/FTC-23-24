/* TFOD_BlueFrontAprilTag - example autonomous mode program for Centerstage

    The program attempts to recognize a Team Prop on a spike more, place the purple pixel on that mark 
        and yellow pixel in the corresponding area on the backdrop.
        
    It makes use of TensorFlow to detect the Team Prop and April Tags to detect the backdrop
    and also uses the IMU for turns and driving backstage.
        
    Starting position - blue alliance, audience side of field, Duplo Team Props
    
    Program flow:
    - Use TensorFlow to detect which spike mark has the Team Prop
    - drive toward indicated spike mark, 
    - backup to deploy the purple pixel on the mark
    - turn to face the front wall April Tag
    - April Tag drive towards the blue field wall
    - use the IMU to turn and face backstage
    - drive towards the backstage area
    - Turn to face the backdrop
    - use code from RobotAutoDriveToAprilTagOmni to drive and line up on area in backdrop
    - deploy yellow pixel using arm
    - park to the right side of backstage to allow alliance partner robot access to the backdrop.
    
    Note: not all options/steps programmed, this only works if it finds the pixel on the left spike mark.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="TFOD_BlueFrontAprilTag")
public class AutonomousProgram extends LinearOpMode
{
    final double DESIRED_DISTANCE = 12; //  this is how close the camera should get to the target (inches)
    final double SPEED_GAIN  =  0.02  ;   // 0.02 Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.02  ;   // 0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.5;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor fL = null;
    private DcMotor fR = null;
    private DcMotor bL = null;
    private DcMotor bR = null;

    private static final boolean USE_WEBCAM = true;
    private int DESIRED_TAG_ID = 10;     // 584 Choose the tag you want to approach or set to -1 for ANY tag.
    private int backdropTagID = 0;
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private int     myExposure  ;
    private int     myGain      ;
    private boolean targetFound  = false;    // Set to true when an AprilTag target is detected

    private WebcamName webcam1;

    private ElapsedTime runtime = new ElapsedTime();

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;
    final String TFOD_MODEL_FILE = "redBlueDuploFar.tflite";
    public static final String[] LABELS = {"blueCone", "redCone"};

    IMU imu;

    TouchSensor touchSensor;  // Touch sensor Object

    // servo variables
    static final double INCREMENT   = 0.08;     // .01 amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.25;     // 0.0 Minimum rotational position

    // Define class members
    Servo armServo, clawLeft, clawRight;

    @Override public void runOpMode()
    {

        boolean propFound      = false;    // set to true when Team Prop found on spike mark
        String propLocation   = "";        // set to left, centre, or right
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        int     currentStep            = 1;

        // Initialize TensorFlow, April Tag, and Vision Portal
        initVisionPortal();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        fL = hardwareMap.get(DcMotor.class, "lF");
        fR = hardwareMap.get(DcMotor.class, "rF");
        bL  = hardwareMap.get(DcMotor.class, "lB");
        bR = hardwareMap.get(DcMotor.class, "rB");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // init IMU
        imu = hardwareMap.get(IMU.class, "imu");
        double targetYaw = 0;

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN; // Which way the logo of the control & expansion hub is
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD; // Which was the usb is on the control hub
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        //visionPortal.setActiveCamera(webcam1);

        // get a reference to our touchSensor object.
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");


        /* Servo Initialization */
        armServo = hardwareMap.get(Servo.class, "armServo");
        clawLeft = hardwareMap.get(Servo.class, "rightClawServo");
        clawRight = hardwareMap.get(Servo.class, "leftClawServo");


        waitForStart();

        visionPortal.stopLiveView();  // turn off liveView while robot moving
        runtime.reset();  // start timer for step 1


        while (opModeIsActive())
        {
            targetFound = false;
            desiredTag  = null;
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            // STEP 1 use Tensorflow to check for Team Prop, timeout after 3 seconds
            if (currentStep==1) {
                if (runtime.milliseconds() < 3000) {
                    List<Recognition> currentRecognitions = tfod.getRecognitions();
                    if (currentRecognitions.size() == 0){
                        sleep(50);  // give TensorFlow time to find the prop before trying again
                    }
                    else {
                        // Step through the list of recognitions and look for team prop
                        for (Recognition recognition : currentRecognitions) {
                            if (recognition.getLeft() < 50) {
                                currentStep = 2;
                                propLocation = "left";
                                backdropTagID = 1;
                                propFound = true;
                                runtime.reset();  // start timer for step 2
                                break;
                            }
                            else if (recognition.getLeft() < 350) {
                                currentStep = 3;
                                propLocation = "centre";
                                backdropTagID = 2;
                                propFound = true;
                                runtime.reset();  // start timer for step 3
                                break;
                            }
                            else {
                                currentStep = 4;
                                propLocation = "right";
                                backdropTagID = 3;
                                propFound = true;
                                runtime.reset();  // start timer for step 4
                                break;
                            }
                        }
                    }
                }
                else {  // Prop not found after timeout, assume Left mark as outside marks harder to detect
                    currentStep = 2;
                    propLocation = "left";
                    backdropTagID = 1;
                    runtime.reset();  // start timer for step 2
                }
            }

            // STEP 2 drive forward to left spike mark
            if (currentStep==2) {
                if (runtime.milliseconds() < 500) {
                    moveRobot(0, 0, 0); // Moving the robot (change later)
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 5;    // backup step
                    runtime.reset();  // start timer for step 3
                }
            }

            // STEP 3 drive forward to center spike mark
            if (currentStep==3) {
                if (runtime.milliseconds() < 1500) {
                    moveRobot(0, 0, 0); // Moving the robot (change later)
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 5;    // backup 
                    runtime.reset();  // start timer for step 5
                }
            }

            // STEP 4 move forward towards right spike mark
            if (currentStep==4) {
                if (runtime.milliseconds() < 500) {
                    moveRobot(0, 0, 0); // Moving the robot (change later)
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 5;
                    runtime.reset();  // start timer for step 5
                }
            }

            // STEP 5 back off, leaving purple pixel on mark
            if (currentStep==5) {
                if (runtime.milliseconds() < 700) {
                    moveRobot(-0.5, 0, 0); // Moving the robot (change later)
                    // Add movement for the claw to drop the pixel
                }
                else {
                    moveRobot(0, 0, 0);
                    visionPortal.setProcessorEnabled(tfod,false);  // turn off TensorFlow
                    currentStep = 6;  // turn to face front wall
                    runtime.reset();  // start timer for step 6
                }
            }

            // STEP 6 turn to heading 90 to face front wall
            if (currentStep==6) {
                if (runtime.milliseconds() < 2500) {
                    imuTurn(-90.0); // Turning angle of the robot (change later)
                    if(targetFound && (Math.abs(desiredTag.ftcPose.yaw - -90) < 7)){
                        moveRobot(0, 0, 0); // stop any moves (robot should not be moving)
                        currentStep = 9; // go backstage
                        runtime.reset();  // start timer for step 9 
                    }
                }
                else {
                    moveRobot(0, 0, 0); // stop any moves
                    currentStep = 7; // prepare to go backstage
                    runtime.reset();  // start timer for step 7
                }
            }

            // STEP 7 drive to April Tag target position
            //  - should end up in tile A2
            if (currentStep==7) {
                if (runtime.milliseconds() < 3000) {
                    aprilTagDrive(DESIRED_TAG_ID, DESIRED_DISTANCE,0.0,-38.0);
                }
                else {
                    moveRobot(0, 0, 0); // stop any moves
                    currentStep = 8; // prepare to go backstage
                    runtime.reset();  // start timer for step 8
                }
            }

            // STEP 8 turn to face backstage
            if (currentStep==8) {
                if (runtime.milliseconds() < 4000) {
                    imuTurn(90);
                    if(targetFound && (Math.abs(desiredTag.ftcPose.yaw - 90) < 7)){
                        moveRobot(0, 0, 0); // stop any moves
                        currentStep = 9; // go backstage
                        runtime.reset();  // start timer for step 9 
                    }
                }
                else {
                    moveRobot(0, 0, 0); // stop any moves
                    currentStep = 9; // go backstage
                    runtime.reset();  // start timer for step 9
                }
            }

            // STEP 9 drive backstage
            if (currentStep==9) {
                if (runtime.milliseconds() < 3500) {
                    moveRobot(0, 0, 0); // Move the robot (change it later)
                }
                else {
                    moveRobot(0, 0, 0);
                    currentStep = 10;    // turn to face backdrop
                    runtime.reset();     // start timer for step 10
                }
            }

            // STEP 10 turn to face backdrop
            if (currentStep==10) {
                if (runtime.milliseconds() < 1250) {
                    imuTurn(45); // Angle of the turn if needed (change this later if needed)
                    if(targetFound && (Math.abs(desiredTag.ftcPose.yaw - 45) < 7)){
                        moveRobot(0, 0, 0); // stop any moves
                        currentStep = 11; // drive to backdrop
                        DESIRED_TAG_ID = backdropTagID;
                        setManualExposure(6, 250);
                        runtime.reset();  // start timer for step 11
                    }
                }
                else {
                    moveRobot(0, 0, 0); // stop any moves
                    currentStep = 11; // drive to backdrop
                    DESIRED_TAG_ID = backdropTagID;
                    setManualExposure(6, 250);
                    runtime.reset();  // start timer for step 11
                }
            }

            // STEP 11 drive to backdrop April Tag
            if (currentStep==11) {
                if (runtime.milliseconds() < 3200) {
                    aprilTagDrive(DESIRED_TAG_ID, /*How far away the robot should be from the april tag*/DESIRED_DISTANCE, 0, 0);
                }
                else {
                    moveRobot(0, 0, 0); // stop any moves
                    currentStep = 12; // move to backdrop
                    runtime.reset();  // start timer for step 12
                }
            }

            // STEP 12 shift right to position arm for backdrop area
            if (currentStep==12) {
                if (runtime.milliseconds() < 600) {
                    moveRobot(0, -0.3, 0);
                }
                else {
                    moveRobot(0, 0, 0); // stop any moves
                    currentStep = 31; // move to backdrop
                    //runtime.reset();  // start timer for step 13
                }
            }

            // STEP 31 touch backdrop, drive until touch sensor
            if (currentStep==31) {
                double elapsedTime = runtime.milliseconds();
                double estimatedDistance = 0.2 * elapsedTime;
                if (estimatedDistance <= DESIRED_DISTANCE) {
                    moveRobot(0, 0, 0);
                    currentStep=32;  // deploy arm
                }
                else {
                    moveRobot(0.2, 0, 0);  // move forward to the backboard
                }
            }

            // STEP 32 deploy arm with yellow pixel to backdrop
            if (currentStep==32) {
                // Code the pixel to be dropped onto the backdrop
                //currentStep==33;
            }

            // STEP 33 retract arm
            if (currentStep==33) {
                // Retract the arm code
                //currentStep==34
            }

            // STEP 34 park, want to back away a little bit and strafe left -X, +Y
            if (currentStep==34) {
                // Move robot so that it can be parked
            }

            telemetry.addData("current step", currentStep);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("prop found", propFound);
            telemetry.addData("prop location", "%s", propLocation);
            telemetry.addData("tag target", DESIRED_TAG_ID);
            telemetry.addData("tag found", targetFound);
            if (targetFound)  {
                telemetry.addData("tag bearning", desiredTag.ftcPose.bearing);
                telemetry.addData("target yaw", desiredTag.ftcPose.yaw);
            }

            telemetry.update();
        }
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }


    // Initialize the Tensorflow vision portal, april tag vision portal
    private void initVisionPortal() {

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam");


        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();
        

        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam1)
                .addProcessor(aprilTag)
                .addProcessor(tfod)
                .build();
    }


    // April Tag Driving.... Driving towards the April Tags
    public void aprilTagDrive(int targetTag, double targetRange, double targetBearing, double targetYaw) {
        double drive, strafe, turn;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        targetFound = false;
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == targetTag)){
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            }
        }

        if (targetFound) {
            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - targetRange);
            double  headingError    = desiredTag.ftcPose.bearing - targetBearing;
            double  yawError        = desiredTag.ftcPose.yaw - targetYaw;

            //    if ((rangeError < 4) && (Math.abs(headingError) < 6) && (Math.abs(yawError) < 6)) {
            //        drive = 0;
            //        turn = 0;
            //        strafe = 0;
            //        currentStep = 31;  // drive to backdrop
            //    }
            //    else {
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
        else {
            drive=0;  strafe=0; turn=0;
        }
        // Apply desired axes motions to the drivetrain.
        moveRobot(drive, strafe, turn);
    }


    /*
    public void imuMove(double powerLevel, /*double duration, double heading) {
        YawPitchRollAngles orientation;
        double turn, headingError;

        orientation = imu.getRobotYawPitchRollAngles();
        headingError    = heading - orientation.getYaw(AngleUnit.DEGREES);
        turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        if (powerLevel < 0) {
            turn = turn * -1;  // reverse turn if going backwards
        }
        moveRobot(powerLevel, 0, turn);
     */

    public void imuTurn(/*double powerLevel, double duration*/ double heading) {
        YawPitchRollAngles orientation;
        double turn, headingError;

        orientation = imu.getRobotYawPitchRollAngles();
        headingError    = heading - orientation.getYaw(AngleUnit.DEGREES);
        turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        moveRobot(0, 0, turn);
    }


    /**
     * Move robot according to desired axes motions
     * Positive X is forward
     * Positive Y is strafe left
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        /* positive values of x move forward
           positive values of y move sideways to the right 
           positive values of yaw rotate clockwise
        */
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        fL.setPower(leftFrontPower);
        fR.setPower(rightFrontPower);
        bL.setPower(leftBackPower);
        bR.setPower(rightBackPower);
        sleep(10);
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}