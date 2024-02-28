package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="ViperSlides-only")
public class ViperSlides extends LinearOpMode {

    private DcMotor leftSlide, rightSlide;
    private int currentPositionLeft, currentPositionRight;
    public void runOpMode() {

        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
        currentPositionLeft = leftSlide.getCurrentPosition();
        currentPositionRight = rightSlide.getCurrentPosition();

        waitForStart();
        while (opModeIsActive()) {
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
                if (currentPositionLeft < 3000 && currentPositionRight < 3000) {
                    leftSlide.setTargetPosition(3000);
                    rightSlide.setTargetPosition(3000);

                    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            } else if (gamepad2.right_bumper) {
                if (currentPositionLeft >= 3000 && currentPositionRight >= 3000) {
                    leftSlide.setTargetPosition(10);
                    rightSlide.setTargetPosition(10);
                    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }

            telemetry.addData("Left Slide Position: ", currentPositionLeft);
            telemetry.addData("Right Slide Position: ", currentPositionRight);

        }
    }
}
