package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.List;
import java.util.concurrent.TimeUnit;


@Autonomous(name="AutonomousSimplifiedClose")
public class AutonomousSimplifiedClose extends LinearOpMode{
    final double DESIRED_DISTANCE = 12; //  this is how close the camera should get to the target (inches)
    final double SPEED_GAIN  =  0.02  ;   // 0.02 Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.02  ;   // 0.01 Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.5;   //  Clip the turn speed to this max value (adjust for your robot)

    private int turnAngle;
    private Odometry odometryPods;

    private DcMotor fL = null;
    private DcMotor fR = null;
    private DcMotor bL = null;
    private DcMotor bR = null;

    private DcMotor odometryPodRight;
    private DcMotor odometryPodLeft;
    private DcMotor odometryPodCenter;
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


    // servo variables
    static final double INCREMENT   = 0.08;     // .01 amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.25;     // 0.0 Minimum rotational position

    // Define class members
    Servo armServo, clawLeft, clawRight;
    private DcMotor leftSlide, rightSlide;

    private int stepNumber;


    @Override public void runOpMode() {
        boolean propFound      = false;    // set to true when Team Prop found on spike mark
        String propLocation   = "";        // set to left, centre, or right
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        int     currentStep;

        fL = hardwareMap.get(DcMotor.class, "lF");
        fR = hardwareMap.get(DcMotor.class, "rF");
        bL  = hardwareMap.get(DcMotor.class, "lB");
        bR = hardwareMap.get(DcMotor.class, "rB");

        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        visionPortal.setActiveCamera(webcam1);

        /* Servo Initialization */
        armServo = hardwareMap.get(Servo.class, "armServo");
        clawLeft = hardwareMap.get(Servo.class, "rightClawServo");
        clawRight = hardwareMap.get(Servo.class, "leftClawServo");

        /* Slide Initialization */
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

        waitForStart();

        visionPortal.stopLiveView();  // turn off liveView while robot moving
        runtime.reset();  // start timer for step 1

        while (opModeIsActive()) {
            targetFound = false;
            desiredTag  = null;

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            for (Recognition recognition : currentRecognitions) {
                
            }
        }
    }


    // Variables
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

    public void moveRobot(double drive, double strafe, double rotate) {
        // Constants for the conversion factor from encoder ticks to distance
        final double TICKS_PER_REV = 1440; // Assuming a motor with 1440 ticks per revolution
        final double WHEEL_DIAMETER_INCHES = 4; // Replace with your wheel diameter in inches
        final double GEAR_RATIO = 1; // Replace with your gear ratio
        final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES; // Circumference = π * diameter

        // Calculate motor powers based on drive, strafe, and rotate values
        double powerFL = drive - strafe + rotate;
        double powerFR = drive + strafe + rotate;
        double powerBL = drive + strafe - rotate;
        double powerBR = drive - strafe - rotate;

        // Convert power values to encoder ticks
        int ticksFL = (int) (powerFL * TICKS_PER_REV / (2 * Math.PI * GEAR_RATIO * WHEEL_CIRCUMFERENCE));
        int ticksFR = (int) (powerFR * TICKS_PER_REV / (2 * Math.PI * GEAR_RATIO * WHEEL_CIRCUMFERENCE));
        int ticksBL = (int) (powerBL * TICKS_PER_REV / (2 * Math.PI * GEAR_RATIO * WHEEL_CIRCUMFERENCE));
        int ticksBR = (int) (powerBR * TICKS_PER_REV / (2 * Math.PI * GEAR_RATIO * WHEEL_CIRCUMFERENCE));

        // Set target positions for the motor encoders
        fL.setTargetPosition(fL.getCurrentPosition() + ticksFL);
        fR.setTargetPosition(fR.getCurrentPosition() + ticksFR);
        bL.setTargetPosition(bL.getCurrentPosition() + ticksBL);
        bR.setTargetPosition(bR.getCurrentPosition() + ticksBR);

        // Set motor run mode to RUN_TO_POSITION
        fL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power for the motors
        fL.setPower(Math.abs(powerFL));
        fR.setPower(Math.abs(powerFR));
        bL.setPower(Math.abs(powerBL));
        bR.setPower(Math.abs(powerBR));
    }
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
