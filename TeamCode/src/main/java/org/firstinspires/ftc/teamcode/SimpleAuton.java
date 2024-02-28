package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="SimpleAuton")
public class SimpleAuton extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;


    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "fL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "fR");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "bL");
        rightBackDrive = hardwareMap.get(DcMotor.class, "bR");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        ElapsedTime runtime = new ElapsedTime();
        while (opModeIsActive()) {
            while (runtime.milliseconds() < 500) {
                moveRobot(-1, 0, 0);
            }
            stopRobot();
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

    private void stopRobot() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
