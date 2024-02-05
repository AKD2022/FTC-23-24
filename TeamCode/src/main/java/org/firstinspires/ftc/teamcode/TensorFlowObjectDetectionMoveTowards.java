package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class TensorFlowObjectDetectionMoveTowards extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private DcMotor fL = null;
    private DcMotor fR = null;
    private DcMotor bL = null;
    private DcMotor bR = null;

    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    private static final String[] LABELS = {
            "teamProp",
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

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else {
                    visionPortal.resumeStreaming();
                }

                // If it finds the cone, it should move forward
                List<Recognition> currentRecognitions = tfod.getRecognitions();
                for (Recognition recognition : currentRecognitions) {
                    moveRobot(10, 0, 0);
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
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.enableLiveView(true);
        builder.addProcessor(tfod);

        visionPortal = builder.build();
    }

    // Telemetry of the Object
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   

    }

    public void moveRobot(double drive, double strafe, double rotate) {
        /* positive values of drive move forward, negative moves back (drive)
           positive values of strafe move sideways to the right, negative moves left (strafe)
           positive values of rotate rotate clockwise, negative rotate counterclockwise (twist)
        */
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
        fL.setPower(leftFrontPower);
        fR.setPower(rightFrontPower);
        bL.setPower(leftBackPower);
        bR.setPower(rightBackPower);
        sleep(10);
    }

}
