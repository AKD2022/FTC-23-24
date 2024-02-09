package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
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
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private DcMotor leftSlide;
    private DcMotor rightSlide;

    private DcMotor fL, fR, bL, bR;
    private static final String TFOD_MODEL_FILE = "model_20240208_132717.tflite";
    //private static final String TFOD_MODEL_ASSET_2 = "model_20240208_132928.tflite";
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
        fL = hardwareMap.get(DcMotor.class, "lF");
        fR = hardwareMap.get(DcMotor.class, "rF");
        bL  = hardwareMap.get(DcMotor.class, "lB");
        bR = hardwareMap.get(DcMotor.class, "rB");


        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);

        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryTfod();


                // Push telemetry to the Driver Station.
                telemetry.update();
                // If it finds the cone, it should move forward
                List<Recognition> currentRecognitions = tfod.getRecognitions();
                for (Recognition recognition : currentRecognitions) {
                    //moveRobot(10, 0, 0);
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

        builder.enableLiveView(true);
        builder.addProcessor(tfod);

        visionPortal = builder.build();
    }

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime totalRuntime = new ElapsedTime();
    boolean objectFound = false;
    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        if (!objectFound) {
            // Turn left
            moveRobot(0, 0, 45); // Turn left (adjust as needed)

            // Now iterate through recognitions to find the object's position
            while (System.currentTimeMillis() - runtime.milliseconds() < 2000) {
                for (Recognition recognitionLeft : currentRecognitions) {
                    if (recognitionLeft != null) {
                        double x = (recognitionLeft.getLeft() + recognitionLeft.getRight()) / 2;
                        double y = (recognitionLeft.getTop() + recognitionLeft.getBottom()) / 2;

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognitionLeft.getLabel(), recognitionLeft.getConfidence() * 100);
                        telemetry.addData("- Position", "%.0f / %.0f", x, y);
                        telemetry.addData("- Size", "%.0f x %.0f", recognitionLeft.getWidth(), recognitionLeft.getHeight());

                        objectFound = true;

                        // Run Autonomous functions for on the left.
                        moveRobot(20, 0, 0); // Drive towards spike mark
                        moveRobot(0, 0, 45); // Turn to face spikeMark

                        return; // Exit the function
                    }
                }
            }


            // Turn right
            moveRobot(0, 0, 180); // Turn right (adjust as needed)
            runtime.reset();

            // Now iterate through recognitions to find the object's position
            while (System.currentTimeMillis() - runtime.milliseconds() < 2000) {
                for (Recognition recognitionRight : currentRecognitions) {
                    if (recognitionRight != null) {
                        double x = (recognitionRight.getLeft() + recognitionRight.getRight()) / 2;
                        double y = (recognitionRight.getTop() + recognitionRight.getBottom()) / 2;

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognitionRight.getLabel(), recognitionRight.getConfidence() * 100);
                        telemetry.addData("- Position", "%.0f / %.0f", x, y);
                        telemetry.addData("- Size", "%.0f x %.0f", recognitionRight.getWidth(), recognitionRight.getHeight());

                        objectFound = true;
                        moveRobot(100, 0, 0);
                        return; // Exit the function
                    }
                }
            }


            // Turn to the center
            moveRobot(0, 0, 45); // Turn to the center
            runtime.reset();

            // Now iterate through recognitions to find the object's position
            while (System.currentTimeMillis() - runtime.milliseconds() < 2000) {
                for (Recognition recognition : currentRecognitions) {
                    if (recognition != null) {
                        double x = (recognition.getLeft() + recognition.getRight()) / 2;
                        double y = (recognition.getTop() + recognition.getBottom()) / 2;

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position", "%.0f / %.0f", x, y);
                        telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

                        objectFound = true;
                        moveRobot(100, 0, 0);
                        return; // Exit the function
                    }
                }
            }

            // If object is not found within 2 seconds, proceed with next action
            // Go to Parking
        } else {
            // Go to Parking
        }
    }



    public void moveRobot(double drive, double strafe, double rotate) {
        final double TICKS_PER_REV = 1440;
        final double WHEEL_DIAMETER_INCHES = 4;
        final double GEAR_RATIO = 1;
        final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;

        double powerFL = drive - strafe + rotate;
        double powerFR = drive + strafe + rotate;
        double powerBL = drive + strafe - rotate;
        double powerBR = drive - strafe - rotate;

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
}
