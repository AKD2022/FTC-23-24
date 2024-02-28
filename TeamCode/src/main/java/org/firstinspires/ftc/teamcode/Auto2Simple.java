package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.w3c.dom.Element;


import java.util.List;

@TeleOp(name="auto2Simple")
public class Auto2Simple extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private Servo leftClaw, rightClaw, arm;
    private DcMotor leftSlide, rightSlide;
    private int currentPositionLeft, currentPositionRight;

    private static final String TFOD_MODEL_FILE = "model_20240218_135347.tflite";
    private static final String[] LABELS = {
            "bluePipe",
            "redPipe"
    };

    // Tensorflow Object Processor
    private TfodProcessor tfod;

    // Vision Portal
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        initTfod();

        /* Drivetrain Init */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "fL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fR");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bL");
        rightBackDrive = hardwareMap.get(DcMotor.class, "bR");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        /* Servo Init */
        leftClaw = hardwareMap.get(Servo.class, "leftClaw"); // 0 on control hub
        rightClaw = hardwareMap.get(Servo.class, "rightClaw"); // 1 on control hub
        arm = hardwareMap.get(Servo.class, "arm"); // 2 control hub

        leftClaw.setPosition(0.5);
        rightClaw.setPosition(0.5);
        rightClaw.setDirection(Servo.Direction.REVERSE);

        /* Viper Slide Init */
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        currentPositionLeft = leftSlide.getCurrentPosition();
        currentPositionRight = rightSlide.getCurrentPosition();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();



        waitForStart();

            while (opModeIsActive()) {
                ElapsedTime runtime = new ElapsedTime();
                    // starts looking at the center
                    runtime.reset();
                    while (runtime.milliseconds() < 2000) {
                        moveRobot(-1, 0, 0);
                    }
                    stopRobot();
                    
            }

    }

    //
    private void initTfod() {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
    }


    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timeToMove = new ElapsedTime();
    int num = 0;
    boolean objectFound = false;
    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

    }

    public void moveRobot(double forwardBackward, double strafe, double rotate) {
        // Check if any parameter has a non-zero value
        if (forwardBackward != 0 || strafe != 0 || rotate != 0) {
            // Calculate individual motor powers based on the parameters
            double leftFrontPower = forwardBackward - strafe - rotate;
            double rightFrontPower = forwardBackward + strafe + rotate;
            double leftBackPower = forwardBackward + strafe - rotate;
            double rightBackPower = forwardBackward - strafe + rotate;

            // Normalize wheel powers to be within the range [-1, 1]
            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send powers to the motors
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        } else {
            stopRobot();
        }
    }

    /* Move Robot Backup */
    public void moveRobotBackup(double drive, double strafe, double rotate) {
        /* SETS POWER */
        // Calculate wheel powers.
        double leftFrontPower = drive + strafe + rotate;
        double rightFrontPower = drive - strafe - rotate;
        double leftBackPower = drive - strafe + rotate;
        double rightBackPower = drive + strafe - rotate;

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
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        sleep(10);
    }


    private void stopRobot() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}