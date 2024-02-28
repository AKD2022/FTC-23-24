package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//Arnesh, Owen
@TeleOp(name="DrivingMain")
public class DrivingMain extends LinearOpMode  {
    private DcMotor fL, fR, bL, bR;
    private DcMotor leftSlide, rightSlide;
    private Servo leftClaw, rightClaw, arm, airplaneLauncher;

    private int currentPositionLeft, currentPositionRight;

    @Override
    public void runOpMode() throws InterruptedException{
        // Drivetrain
        fL = hardwareMap.get(DcMotor.class, "fL"); //port 0
        fR = hardwareMap.get(DcMotor.class, "fR"); //port 1
        bL = hardwareMap.get(DcMotor.class, "bL"); //port 2
        bR = hardwareMap.get(DcMotor.class, "bR"); //port 3

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);

        // Viper Slides
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        currentPositionLeft = leftSlide.getCurrentPosition();
        currentPositionRight = rightSlide.getCurrentPosition();

        // Claw
        leftClaw = hardwareMap.get(Servo.class, "leftClaw"); // 0 on control hub
        rightClaw = hardwareMap.get(Servo.class, "rightClaw"); // 1 on control hub
        arm = hardwareMap.get(Servo.class, "arm"); // 2 control hub
        arm.setDirection(Servo.Direction.REVERSE);
        arm.setPosition(0.8);
        leftClaw.setPosition(0.5);
        rightClaw.setPosition(0.5);
        rightClaw.setDirection(Servo.Direction.REVERSE);

        /* Airplane Launcher init */
        airplaneLauncher = hardwareMap.get(Servo.class, "airplaneServo"); // 3 on control hub

        waitForStart();
        while(opModeIsActive()) {
            // Driving
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Calculate motor powers
            double frontLeftPower = drive + strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double backLeftPower = drive - strafe + rotate;
            double backRightPower = drive + strafe - rotate;

            // Normalize motor powers
            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Set motor powers
            fL.setPower(frontLeftPower);
            fR.setPower(frontRightPower);
            bL.setPower(backLeftPower);
            bR.setPower(backRightPower);


            telemetry.addData("Front Left Power", frontLeftPower);
            telemetry.addData("Front Right Power", frontRightPower);
            telemetry.addData("Back Left Power", backLeftPower);
            telemetry.addData("Back Right Power", backRightPower);
            telemetry.update();

            // Viper Slides
            if (gamepad2.left_trigger != 0) {
                leftSlide.setPower(1.0);
                rightSlide.setPower(1.0);
            } else if (gamepad2.right_trigger != 0) {
                leftSlide.setPower(-1.0);
                rightSlide.setPower(-1.0);
            } else {
                leftSlide.setPower(0.0);
                rightSlide.setPower(0.0);
            }

            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if (gamepad2.left_bumper) {
                if (currentPositionLeft < 3000 && currentPositionRight < -3000) {
                    leftSlide.setTargetPosition(3000);
                    rightSlide.setTargetPosition(3000);

                    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            } else if (gamepad2.right_bumper) {
                if (currentPositionLeft >= 3000 && currentPositionRight >= -3000) {
                    leftSlide.setTargetPosition(10);
                    rightSlide.setTargetPosition(10);

                    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }

            telemetry.addData("Left Slide Position: ", currentPositionLeft);
            telemetry.addData("Right Slide Position: ", currentPositionRight);

            // Claw

            if (gamepad2.x) {
                leftClaw.setPosition(0.5);
                rightClaw.setPosition(0.5);
            } else if (gamepad2.y) {
                leftClaw.setPosition(0.68);
                rightClaw.setPosition(0.68);
            } else if (gamepad2.a) {
                arm.setPosition(0.8); // adjust later
            } else if (gamepad2.b) {
                arm.setPosition(0.5); // adjust later
            }

            /* Airplane Launcher */
            if (gamepad1.x) {
                airplaneLauncher.setPosition(1.0); // launch the plane
            } else if (gamepad1.y) {
                airplaneLauncher.setPosition(0.0); // reset the servo
            }


        }
    }
}