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

@TeleOp(name="autoFinal")
public class TensorFlowObjectDetectionMoveTowards extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private Servo leftClaw, rightClaw, arm;
    private DcMotor leftSlide, rightSlide;
    private int currentPositionLeft, currentPositionRight;

    private static final String TFOD_MODEL_FILE = "model_20240208_132717.tflite";
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

        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else {
                    visionPortal.resumeStreaming();
                }

                // If it finds the cone, it should move forward
                List<Recognition> currentRecognitions = tfod.getRecognitions();
                for (Recognition recognition : currentRecognitions) {

                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

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

        if (!objectFound) {
            // starts looking at the center
            runtime.reset();

            while (runtime.milliseconds() < 5000) {
                for (Recognition recognition : currentRecognitions) {
                    if (recognition != null) {
                        double x = (recognition.getLeft() + recognition.getRight()) / 2;
                        double y = (recognition.getTop() + recognition.getBottom()) / 2;

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position", "%.0f / %.0f", x, y);
                        telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

                        objectFound = true;
                        num = 1; // center
                        break;
                    }
                }

                if (objectFound) {
                    num = 1;
                    break;
                }
            }


            // Turn left
            timeToMove.reset();
            while (timeToMove.milliseconds() < 100) {
                moveRobot(0, 0, 0.5);
            }
            stopRobot();
            timeToMove.reset();
            runtime.reset();

            // Now iterate through recognitions to find the object's position
            while (runtime.milliseconds() < 5000) {
                for (Recognition recognition : currentRecognitions) {
                    if (recognition != null) {
                        double x = (recognition.getLeft() + recognition.getRight()) / 2;
                        double y = (recognition.getTop() + recognition.getBottom()) / 2;

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position", "%.0f / %.0f", x, y);
                        telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

                        objectFound = true;
                        num = 2; // left
                        break;
                    }
                }

                if (objectFound) {
                    num = 2;
                    break;
                }
            }


            // Turn to the right
            timeToMove.reset();
            while (timeToMove.milliseconds() < 100) {
                moveRobot(0, 0, -0.5);
            }
            stopRobot();
            timeToMove.reset();
            runtime.reset();

            // Now iterate through recognitions to find the object's position
            while (runtime.milliseconds() < 5000) {
                for (Recognition recognition : currentRecognitions) {
                    if (recognition != null) {
                        double x = (recognition.getLeft() + recognition.getRight()) / 2;
                        double y = (recognition.getTop() + recognition.getBottom()) / 2;

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position", "%.0f / %.0f", x, y);
                        telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

                        objectFound = true;
                        num = 3; // right
                        break;
                    }
                }

                if (objectFound) {
                    num = 3;
                    break;
                }
            }
        } else {
            // Go to Parking
        }

        if (objectFound && num == 3) {
            rightSpikeMark();
        } else if (objectFound && num == 2) {
            leftSpikeMark();
        } else if (objectFound && num == 1) {
            centerSpikeMark();
        }
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

    private void leftSpikeMark() {
        ElapsedTime centerElapsedTime = new ElapsedTime();
        centerElapsedTime.reset();

        /* Move Forward */
        while (centerElapsedTime.milliseconds() < 1000) {
            moveRobot(1, 0, 0);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Open claw with purple pixel */
        while (centerElapsedTime.milliseconds() < 500) {
            leftClaw.setPosition(0.65);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Close Claw */
        while (centerElapsedTime.milliseconds() < 500) {
            leftClaw.setPosition(0.35);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Turn towards backboard */
        while (centerElapsedTime.milliseconds() < 200) {
            moveRobot(0, 0, 150);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Go to the left column */
        while (centerElapsedTime.milliseconds() < 50) {
            moveRobot(0, -1, 0);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Go Towards the backboard */
        while (centerElapsedTime.milliseconds() < 1200) {
            moveRobot(1, 0, 0);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Move the Viper Slide up */
        while (centerElapsedTime.milliseconds() < 1000) {
            /* Setting the height that it needs to go to */
            leftSlide.setTargetPosition(3000);
            rightSlide.setTargetPosition(3000);

            /* Set the RUN TO POSITION so it runs to 3000 */
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Setting the power of the slides */
            leftSlide.setPower(0.5);
            rightSlide.setPower(0.5);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Move Arm */
        while (centerElapsedTime.milliseconds() < 500) {
            arm.setDirection(Servo.Direction.REVERSE);
            arm.setPosition(0.75); // Change to what is in the code
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Open Claw to drop the other pixel in */
        while (centerElapsedTime.milliseconds() < 500) {
            rightClaw.setPosition(0.75);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Close Claw */
        while (centerElapsedTime.milliseconds() < 500) {
            rightClaw.setPosition(0.35);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Reset Arm */
        while (centerElapsedTime.milliseconds() < 500) {
            arm.setDirection(Servo.Direction.REVERSE);
            arm.setPosition(0.15); // Change to what is in the code
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Move the Viper Slide Back Down */
        while (centerElapsedTime.milliseconds() < 1000) {
            /* Setting the height that it needs to go to */
            leftSlide.setTargetPosition(150);
            rightSlide.setTargetPosition(150);

            /* Set the RUN TO POSITION so it runs to 3000 */
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Setting the power of the slides */
            leftSlide.setPower(0.5);
            rightSlide.setPower(0.5);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Strafe towards the Parking */
        while (centerElapsedTime.milliseconds() < 750) {
            moveRobot(0, 1, 0);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Move Back A bit */
        while (centerElapsedTime.milliseconds() < 100) {
            moveRobot(-1, 0, 0);
        }
        sleep(50);
        centerElapsedTime.reset();
        moveRobot(0, 0, 0);
        stopRobot();
    }


    private void rightSpikeMark() {
        ElapsedTime centerElapsedTime = new ElapsedTime();
        centerElapsedTime.reset();

        /* Move Forward */
        while (centerElapsedTime.milliseconds() < 1000) {
            moveRobot(1, 0, 0);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Open claw with purple pixel */
        while (centerElapsedTime.milliseconds() < 500) {
            leftClaw.setPosition(0.65);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Close Claw */
        while (centerElapsedTime.milliseconds() < 500) {
            leftClaw.setPosition(0.35);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Turn towards backboard */
        while (centerElapsedTime.milliseconds() < 200) {
            moveRobot(0, 0, 30);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Go to the right column */
        while (centerElapsedTime.milliseconds() < 50) {
            moveRobot(0, 1, 0);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Go Towards the backboard */
        while (centerElapsedTime.milliseconds() < 750) {
            moveRobot(1, 0, 0);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Move the Viper Slide up */
        while (centerElapsedTime.milliseconds() < 1000) {
            /* Setting the height that it needs to go to */
            leftSlide.setTargetPosition(3000);
            rightSlide.setTargetPosition(3000);

            /* Set the RUN TO POSITION so it runs to 3000 */
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Setting the power of the slides */
            leftSlide.setPower(0.5);
            rightSlide.setPower(0.5);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Move Arm */
        while (centerElapsedTime.milliseconds() < 500) {
            arm.setDirection(Servo.Direction.REVERSE);
            arm.setPosition(0.75); // Change to what is in the code
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Open Claw to drop the other pixel in */
        while (centerElapsedTime.milliseconds() < 500) {
            rightClaw.setPosition(0.75);
        }
        sleep(50);
        centerElapsedTime.reset();

        /* Close Claw */
        while (centerElapsedTime.milliseconds() < 500) {
            rightClaw.setPosition(0.35);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Reset Arm */
        while (centerElapsedTime.milliseconds() < 500) {
            arm.setDirection(Servo.Direction.REVERSE);
            arm.setPosition(0.15); // Change to what is in the code
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Move the Viper Slide Back Down */
        while (centerElapsedTime.milliseconds() < 1000) {
            /* Setting the height that it needs to go to */
            leftSlide.setTargetPosition(150);
            rightSlide.setTargetPosition(150);

            /* Set the RUN TO POSITION so it runs to 3000 */
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Setting the power of the slides */
            leftSlide.setPower(0.5);
            rightSlide.setPower(0.5);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Strafe towards the Parking */
        while (centerElapsedTime.milliseconds() < 750) {
            moveRobot(0, 1, 0);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Move Back A bit */
        while (centerElapsedTime.milliseconds() < 100) {
            moveRobot(-1, 0, 0);
        }
        sleep(50);
        centerElapsedTime.reset();
        moveRobot(0, 0, 0);
        stopRobot();
    }

    private void centerSpikeMark() {
        ElapsedTime centerElapsedTime = new ElapsedTime();
        centerElapsedTime.reset();

        /* Move Forward */
        while (centerElapsedTime.milliseconds() < 1000) {
            moveRobot(1, 0, 0);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Open claw with purple pixel */
        while (centerElapsedTime.milliseconds() < 500) {
            leftClaw.setPosition(0.65);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Close Claw */
        while (centerElapsedTime.milliseconds() < 500) {
            leftClaw.setPosition(0.35);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Turn towards backboard */
        while (centerElapsedTime.milliseconds() < 200) {
            moveRobot(0, 0, 45);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Go to the center column */
        while (centerElapsedTime.milliseconds() < 50) {
            moveRobot(0, 1, 0);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Go Towards the backboard */
        while (centerElapsedTime.milliseconds() < 1000) {
            moveRobot(1, 0, 0);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Move the Viper Slide up */
        while (centerElapsedTime.milliseconds() < 1000) {
            /* Setting the height that it needs to go to */
            leftSlide.setTargetPosition(3000);
            rightSlide.setTargetPosition(3000);

            /* Set the RUN TO POSITION so it runs to 3000 */
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Setting the power of the slides */
            leftSlide.setPower(0.5);
            rightSlide.setPower(0.5);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Move Arm */
        while (centerElapsedTime.milliseconds() < 500) {
            arm.setDirection(Servo.Direction.REVERSE);
            arm.setPosition(0.75); // Change to what is in the code
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Open Claw to drop the other pixel in */
        while (centerElapsedTime.milliseconds() < 200) {
            rightClaw.setPosition(0.75);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Close Claw */
        while (centerElapsedTime.milliseconds() < 200) {
            rightClaw.setPosition(0.35);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Reset Arm */
        while (centerElapsedTime.milliseconds() < 500) {
            arm.setDirection(Servo.Direction.REVERSE);
            arm.setPosition(0.15); // Change to what is in the code
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Move the Viper Slide Back Down */
        while (centerElapsedTime.milliseconds() < 1000) {
            /* Setting the height that it needs to go to */
            leftSlide.setTargetPosition(150);
            rightSlide.setTargetPosition(150);

            /* Set the RUN TO POSITION so it runs to 3000 */
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Setting the power of the slides */
            leftSlide.setPower(0.5);
            rightSlide.setPower(0.5);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Strafe towards the Parking */
        while (centerElapsedTime.milliseconds() < 500) {
            moveRobot(0, 1, 0);
        }
        sleep(50);
        centerElapsedTime.reset();
        stopRobot();

        /* Move Back A bit */
        while (centerElapsedTime.milliseconds() < 100) {
            moveRobot(-1, 0, 0);
        }
        sleep(50);
        centerElapsedTime.reset();
        moveRobot(0, 0, 0);
        stopRobot();
    }

}