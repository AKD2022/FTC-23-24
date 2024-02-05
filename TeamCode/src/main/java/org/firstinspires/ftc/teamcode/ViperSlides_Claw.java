package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ViperSlides_Claw extends LinearOpMode {

    private Servo leftClaw, rightClaw, arm;
    private DcMotor leftSlide, rightSlide;

    private int currentPositionLeft, currentPositionRight;

    public void runOpMode() {
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        arm = hardwareMap.get(Servo.class, "arm");

        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

        leftClaw.setPosition(0.35);
        rightClaw.setPosition(0.35);
        arm.setPosition(0.15);

        currentPositionLeft = leftSlide.getCurrentPosition();
        currentPositionRight = rightSlide.getCurrentPosition();

        while(opModeIsActive()) {
            /* Claw */
            if (gamepad2.x) {
                leftClaw.setPosition(0.35);
                rightClaw.setPosition(0.35);
            } else if (gamepad2.y) {
                leftClaw.setPosition(0.75);
                rightClaw.setPosition(0.75);
            } else if (gamepad2.a) {
                arm.setPosition(0.75); // adjust later
            } else if (gamepad2.b) {
                arm.setPosition(0.15); // adjust later
            }

            /* Slides */
            if (gamepad2.left_trigger != 0) {
                leftSlide.setPower(1.0);
                rightSlide.setPower(1.0);
            } else if (gamepad2.right_trigger != 0) {
                leftSlide.setPower(-1.0);
                rightSlide.setPower(-1.0);
            }

            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if (gamepad2.left_bumper) {
                if (currentPositionLeft < 6000 && currentPositionRight < 6000) {
                    leftSlide.setTargetPosition(6000); // adjust later
                    rightSlide.setTargetPosition(6000); // adjust later
                }
            } else if (gamepad2.right_bumper) {
                if (currentPositionLeft >= 6000 && currentPositionRight >= 6000) {
                    leftSlide.setTargetPosition(10); // adjust later
                    rightSlide.setTargetPosition(10); // adjust later
                }
            }

            telemetry.addData("Left Slide Position: ", currentPositionLeft);
            telemetry.addData("Right Slide Position: ", currentPositionRight);
        }
    }

}
