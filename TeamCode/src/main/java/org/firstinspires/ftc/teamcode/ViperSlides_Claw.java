package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Viper_Claw")
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
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftClaw = hardwareMap.get(Servo.class, "leftClaw"); // 0 on control hub
        rightClaw = hardwareMap.get(Servo.class, "rightClaw"); // 1 on control hub
        arm = hardwareMap.get(Servo.class, "arm"); // 2 control hub

        leftClaw.setPosition(0.35);
        rightClaw.setPosition(0.35);

        rightClaw.setDirection(Servo.Direction.REVERSE);
        arm.setDirection(Servo.Direction.REVERSE);

        arm.setPosition(0.8);

        currentPositionLeft = leftSlide.getCurrentPosition();
        currentPositionRight = rightSlide.getCurrentPosition();

        waitForStart();

        while(opModeIsActive()) {
            /* Claw */
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

            /* Slides */
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
                    leftSlide.setTargetPosition(3000); // adjust later
                    rightSlide.setTargetPosition(3000); // adjust later

                    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            } else if (gamepad2.right_bumper) {
                if (currentPositionLeft >= 3000 && currentPositionRight >= 3000) {
                    leftSlide.setTargetPosition(10); // adjust later
                    rightSlide.setTargetPosition(10); // adjust later

                    leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }

            telemetry.addData("Left Slide Position: ", currentPositionLeft);
            telemetry.addData("Right Slide Position: ", currentPositionRight);
        }
    }

}
